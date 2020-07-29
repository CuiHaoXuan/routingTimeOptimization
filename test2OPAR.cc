/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mohammed Gharib <mohammed.algharib@nau.edu>
 *
 */

//
// This example simulate an UAV network, with nWifi UAV agents.
// Agents are randomly positioned in an X*Y*Z area and moves 
// according to a chosen mobility model. The mobility models could be:
// 1-RWP, 2-GaussMarkov
// They communicate through a wifi-based ad hoc network. 
// The mobility model has chosen by the parameter protocol and could be:
// 1-OLSR, 2-AODV, 3-DSDV, 4-DSR. 
// Author used manet-routing-compare.cc and tcp-large-transfer.cc
// as its baseline examples and developed his code based on them.

/* 
//  Usage (e.g.): ./waf --run UAVrouting
*/

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <bits/stdc++.h>

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/wifi-module.h"


using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("OPAR");

void setupMobility(double, double,double,NodeContainer,uint32_t,int64_t);
YansWifiPhyHelper setupWifiPhy(double);
Ipv4InterfaceContainer setupIP(NetDeviceContainer);
void setupMobilityTrack(NodeContainer);
void updateLifetimes();
double distance(uint32_t,uint32_t);
double lifeTime(uint32_t,uint32_t);
void traceUAVpositions();
void BFS(double **,uint32_t,uint32_t,int *);//Breadth First Search
double objectiveValue(int *);
void removeLowestLifetimes(double **, int *);
int routLen(int *);

static double txp=7.5;
static double TrRange=138;
static const uint16_t port = 50000;
static const uint32_t totalRxBytes = 5000000;
static Ipv4InterfaceContainer adhocInterfaces;
static NetDeviceContainer adhocDevices;
static NodeContainer adhocNodes;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1040;
static uint8_t data[writeSize];
static MobilityHelper mobilityAdhoc;
static int const nWifi=50;//number of UAVs
static double linkLifetime[nWifi][nWifi]={};//initilize the linkLifetime matix with all zeros 
static Ptr<Socket> mobilityTrackingSocket[nWifi];
static double posMatrix[nWifi][12]={};//each row includes three sets of (time,x,y,z) 
static int traceCount=0;
static double totalTime=100;
static const double pathLenWeight=0.5;// 0 <= pathLenWeight <= 1. pathLenWeight=1 leads to shortest path, pathLenWeight=0 leads to the path with longest lifeTime
static const double pathLifetimeWeight=1-pathLenWeight;


class Flow
{
public:
  Flow();
  void setupConnection(Flow*,uint32_t,uint32_t,uint16_t,Ipv4InterfaceContainer,NodeContainer);

  double throughput;
  double FCT;//flow completion time
  bool successfullyTerminated;
  uint32_t source;
  uint32_t sink;
  int flowNo;

private:
  uint32_t currentTxBytes;
  uint32_t currentTxPackets;
  uint32_t currentRxBytes;
  uint32_t currentRxPackets;
  int pathLen;
  int rout[nWifi];
  bool packetReceived;
  
  Ptr<Socket> sourceSocket[nWifi];
  Ptr<Socket> sinkSocket[nWifi];

  void WriteUntilBufferFull (Ptr<Socket>, uint32_t);
  void accept(Ptr<Socket>,const ns3::Address&);
  void ReceivePacket (Ptr<Socket>);
  void CwndTracer (uint32_t , uint32_t );
  void findRout();
  int posInRout(int );
  void updatePathIfNotAlive(Flow*);
  void closeIntermediateSockets();
  void setupSockets(Flow*);
};

Flow::Flow()
  : throughput(0),
    FCT(0),
    successfullyTerminated(false),
    currentTxBytes(0),
    currentTxPackets(0),
    currentRxBytes(0),
    currentRxPackets(0),
    pathLen(0),
    packetReceived(false)
{
   for(int i=0;i<nWifi;i++)
       this->rout[i]=-1;
}

//###################### main() ###########################
int main (int argc, char *argv[])
{
  int nSinks=1;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  uint32_t mobilityModel=1;//1-RWP, 2-GaussMarkov   

  double X=300.0;
  double Y=1500.0;
  double Z=50.0;
  
  CommandLine cmd;
  cmd.AddValue("nSinks", "No. of sinks to echo", nSinks);
  cmd.AddValue("streamIndex", "Stream Index to echo", streamIndex);
  cmd.AddValue("totalTime", "Total time to echo", totalTime);
  cmd.AddValue("mobilityModel", "Mobility model to echo (1-RWP, 2-GaussMarkov): ", mobilityModel);
  //cmd.AddValue("nWifi","No. of UAVs to echo",nWifi);
  cmd.AddValue("txp","Transmition power to echo",txp);
  cmd.AddValue("X","Area width to echo",X);
  cmd.AddValue("Y","Area Length to echo",Y);
  cmd.AddValue("Z","Area height to echo",Z);
  cmd.Parse (argc, argv);

  std::string phyMode ("DsssRate11Mbps");
  const std::string rate="2048bps";

  std::string mobilityName;

switch (mobilityModel)
    {
    case 1:
      mobilityName = "RWP";
      break;
    case 2:
      mobilityName = "G-M";
      break;
    default:
      NS_FATAL_ERROR ("No such model:" << mobilityModel);
    }

  std::cout<<"OPAR, total time: "<<totalTime<<", pathLenWeight: "<<pathLenWeight<<", no. UAVs: "<<nWifi<<", no. Conections: "<<nSinks<<std::endl;
  std::cout<<"X: "<<X<<", Y: "<<Y<<", Z: "<<Z<<", Mobility model: "<<mobilityName<<", stream Index: "<<streamIndex<<std::endl; 

  std::stringstream ss;
  ss<<"traceFiles/OPAR_UAV"<<nWifi<<"Con"<<nSinks<<"_"<<mobilityName<< "_"<<streamIndex;
  std::string tr_name (ss.str ());
  
  NS_LOG_UNCOND ("Starting the simulation...");

  // initialize the tx buffer.
  for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }
    
    ns3::PacketMetadata::Enable ();
  // Here, we will explicitly create the nodes. 
  // This will be used to install the network
  // interfaces and connect them with a channel.
  adhocNodes.Create (nWifi);

  // We create the channels first without any IP addressing information
  // First make and configure the helper, so that it will put the appropriate
  // attributes on the network interfaces and channels we are about to install.
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  //disable rate control  
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  //setup wifi physical attributes
  YansWifiPhyHelper wifiPhy; 
  wifiPhy= setupWifiPhy(txp);  
  NS_LOG_UNCOND ("Setting up wifi physical attributes...");
  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

  // Add a mac 
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");  
  
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  //setup mobility
  setupMobility(X,Y,Z,adhocNodes,mobilityModel,streamIndex);  
  NS_LOG_UNCOND ("Setting up Mobility...");

  InternetStackHelper internet;
  internet.Install (adhocNodes);
  NS_LOG_UNCOND ("Setting up routing protocol...");
  // Later, we add IP addresses.
  adhocInterfaces=setupIP(adhocDevices);
  NS_LOG_UNCOND ("Setting up IP address for nodes...");

  setupMobilityTrack(adhocNodes);
 
  ///////////////////////////////////////////////////////////////////////////
  // Simulation 1
  //
  // Send 2000000 bytes over a connection to server port 50000 at time 0
  // Should observe SYN exchange, a lot of data segments and ACKS, and FIN 
  // exchange.  FIN exchange isn't quite compliant with TCP spec (see release
  // notes for more info)
  //
  ///////////////////////////////////////////////////////////////////////////

  // Create the connections...

  Flow UAVflow[nSinks];
  std::cout<<"Setting up "<<nSinks<<" connections..."<<std::endl;
  
  for (int i = 0; i < nSinks; i++)
     {
        UAVflow[i].flowNo=i;          
        //UAVflow[i].source=i*2;
        //UAVflow[i].sink=i*2+1;     
        UAVflow[i].setupConnection(&UAVflow[i],i*2,i*2+1,port,adhocInterfaces,adhocNodes);
     }
 
  // One can toggle the comment for the following line on or off to see  
  // the effects of finite send buffer modelling.  One can also change 
  // the size of said buffer.

  //sourceSocket->SetAttribute("SndBufSize", UintegerValue(4096));

  //Ask for ASCII and pcap traces of network traffic
  
  AsciiTraceHelper ascii;
  Ptr<FlowMonitor> flowmon;

  wifiPhy.EnablePcapAll (tr_name);
  wifiPhy.EnableAsciiAll (ascii.CreateFileStream (tr_name+".tr"));
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  AnimationInterface anim (tr_name+".xml");
  anim.SetMaxPktsPerTraceFile (200000000);

  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll (); 
  
  // Finally, set up the simulator to run.  The 'totalTime' second hard limit is a
  // failsafe in case some change above causes the simulation to never end
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();
  
  flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), true, true);

  NS_LOG_UNCOND ("End of simulation...");
  double throughput=0;
  double FCT=0;//flow completion time
  int count=0;
  for (int i = 0; i < nSinks; i++)
     {
         if(UAVflow[i].successfullyTerminated)
           {
              count++;
              throughput+=UAVflow[i].throughput;
              FCT+=UAVflow[i].FCT;
              std::cout<<"Flow no. "<<UAVflow[i].flowNo<<":  Throughput= "<<UAVflow[i].throughput<<",  FCT= "<<UAVflow[i].FCT<<std::endl;
            }
      }
  std::cout<<count<< " flows out of total "<<nSinks<<" flows completed successfully."<<std::endl;
  std::cout<<"Average throughput: "<<throughput/count<<", Average FCT: "<<FCT/count<<std::endl; 
  Simulator::Destroy ();
return count;
  
}
//###################### end of main() ###########################
void 
Flow::closeIntermediateSockets()
{
  for(int i=0;i<nWifi;i++)
     {
       this->sourceSocket[i]->ShutdownSend();
       this->sinkSocket[i]->ShutdownRecv();
     }
}

void 
BFS(double netGraph[nWifi][nWifi],uint32_t src,uint32_t dst, int *path)
{
  int dis[nWifi];//the distance from the src
  int predecessor[nWifi];//the predecessor of each node toward the src
  bool reachDst=false;
  bool noPath=false;
  int current=src;
  list<int> currentLevel;
  for(int i=0;i<nWifi;i++)//marking all vertices as unvisited
     {
       predecessor[i]=-1;
       dis[i]=-1;
     }
  dis[src]=0;
  while(!reachDst && !noPath)
   {
     for(uint32_t i=0;i<nWifi;i++)
        {
          if(netGraph[current][i]>0)
             {
               if(i==dst)
                 {
                   reachDst=true;
                 }
               if(predecessor[i]==-1)// it is not seen yet
                 {
                   currentLevel.push_back(i);
                   predecessor[i]=current;
                   dis[i]=dis[current]+1;
                 }
             }           
        } 
     if(!currentLevel.empty())
        {  
          current=currentLevel.front();
          currentLevel.pop_front();
        }
     else
       noPath=true;
   }
  if(!reachDst)//there is no path
    {
      path[0]=-1;
    }
  else
    {
      path[dis[dst]]=dst;
      for(int i=dis[dst];i>0;i--)
         {
           path[i-1]=predecessor[path[i]];
         }
    }
}

double 
objectiveValue(int *path)
{
  double objective=0;
  double lowestLifetime =totalTime;
  int len=routLen(path);
  for(int i=0;i<len;i++)
     {
       if(linkLifetime[path[i]][path[i+1]]<lowestLifetime)
          {
            lowestLifetime=linkLifetime[path[i]][path[i+1]];
          } 
     }  
  objective=pathLenWeight*len+pathLifetimeWeight/lowestLifetime;
  return objective;
}

void 
removeLowestLifetimes(double netGraph[nWifi][nWifi], int *path)
{
  int len=routLen(path);
  double lowestLifetime=totalTime;
  for(int i=0;i<len;i++)
     {
       if(netGraph[path[i]][path[i+1]]<lowestLifetime)
          {
            lowestLifetime=netGraph[path[i]][path[i+1]];
          } 
     }
  for(int i=0;i<nWifi;i++)
     {
       for(int j=0;j<nWifi;j++)
          {
            if(netGraph[i][j]<=lowestLifetime)
               {
                 netGraph[i][j]=0;
               }
          }
     }
}


void 
Flow::updatePathIfNotAlive(Flow* m_flow)
{
  if(!(this->successfullyTerminated))
    {
      if(!(this->packetReceived))
        { 
          this->setupConnection(m_flow,this->source,this->sink, port,adhocInterfaces,adhocNodes);          
        }
      else
        {
          this->packetReceived=false;
          Simulator::Schedule (Seconds (1.0), &Flow::updatePathIfNotAlive,m_flow,m_flow);
         }   
      } 
}

void 
traceUAVpositions()
{
  Vector currentPos;
  Ptr<MobilityModel> mobility;
  for(int i=0;i<nWifi;i++)
     {
       mobility = mobilityTrackingSocket[i]->GetNode()->GetObject<MobilityModel>();
       currentPos=mobility->GetPosition();

       posMatrix[i][0]=posMatrix[i][4];
       posMatrix[i][1]=posMatrix[i][5];
       posMatrix[i][2]=posMatrix[i][6];
       posMatrix[i][3]=posMatrix[i][7];

       posMatrix[i][4]=posMatrix[i][8];
       posMatrix[i][5]=posMatrix[i][9];
       posMatrix[i][6]=posMatrix[i][10];
       posMatrix[i][7]=posMatrix[i][11];

       posMatrix[i][8]=double(Simulator::Now ().GetSeconds ());
       posMatrix[i][9]=currentPos.x;
       posMatrix[i][10]=currentPos.y;
       posMatrix[i][11]=currentPos.z;
     }
 traceCount++;
 if(traceCount>=3)
   {
     Simulator::Schedule (Seconds (1.0), &traceUAVpositions);
   }
}

double 
lifeTime(uint32_t src,uint32_t dst)
{  
  double time=0;//the link lifetime
  bool flag=true;//Bisection flag which means repeat the process until

  double alpha_src=0;
  double theta_src=0;
  double vel1_src=0;
  double vel2_src=0; 
  double accel_src=0;

  double alpha_dst=0;
  double theta_dst=0;
  double vel1_dst=0;
  double vel2_dst=0; 
  double accel_dst=0;

if ((posMatrix[src][9]-posMatrix[src][5])==0)//to skip nan
{
  alpha_src=0;
}
else
{
  alpha_src=atan((posMatrix[src][10]-posMatrix[src][6])/(posMatrix[src][9]-posMatrix[src][5]));//azimuthal angel
}

if ((posMatrix[src][11]-posMatrix[src][7])==0)//to skip nan
{
  theta_src=0;
}
else
{
  theta_src=atan(sqrt(pow((posMatrix[src][9]-posMatrix[src][5]),2)+pow((posMatrix[src][10]-posMatrix[src][6]),2))/(posMatrix[src][11]-posMatrix[src][7]));//polar angel
}

if ((posMatrix[src][4]-posMatrix[src][0])==0)//to skip nan
{
  vel1_src=0;
}
else
{
  vel1_src=sqrt(pow((posMatrix[src][5]-posMatrix[src][1]),2)+pow((posMatrix[src][6]-posMatrix[src][2]),2)+pow((posMatrix[src][7]-posMatrix[src][3]),2))/(posMatrix[src][4]-posMatrix[src][0]);//velocity of the source node between t0 and t1
}

if ((posMatrix[src][8]-posMatrix[src][4])==0)//to skip nan
{
  vel2_src=0;
}
else
{
  vel2_src=sqrt(pow((posMatrix[src][9]-posMatrix[src][5]),2)+pow((posMatrix[src][10]-posMatrix[src][6]),2)+pow((posMatrix[src][11]-posMatrix[src][7]),2))/(posMatrix[src][8]-posMatrix[src][4]);//velocity of the source node between t1 and t2
}

if ((posMatrix[src][8]-posMatrix[src][0])==0)//to skip nan
{
  accel_src=0;
}
else
{
  accel_src=(vel2_src-vel1_src)/(posMatrix[src][8]-posMatrix[src][0]);//acceleration
}

if ((posMatrix[dst][9]-posMatrix[dst][5])==0)//to skip nan
{
  alpha_dst=0;
}
else
{
  alpha_dst=atan((posMatrix[dst][10]-posMatrix[dst][6])/(posMatrix[dst][9]-posMatrix[dst][5]));//azimuthal angel
}

if ((posMatrix[dst][11]-posMatrix[dst][7])==0)//to skip nan
{
  theta_dst=0;
}
else
{
  theta_dst=atan(sqrt(pow((posMatrix[dst][9]-posMatrix[dst][5]),2)+pow((posMatrix[dst][10]-posMatrix[dst][6]),2))/(posMatrix[dst][11]-posMatrix[dst][7]));//polar angel
}

if ((posMatrix[dst][4]-posMatrix[dst][0])==0)//to skip nan
{
  vel1_dst=0;
}
else
{
  vel1_dst=sqrt(pow((posMatrix[dst][5]-posMatrix[dst][1]),2)+pow((posMatrix[dst][6]-posMatrix[dst][2]),2)+pow((posMatrix[dst][7]-posMatrix[dst][3]),2))/(posMatrix[dst][4]-posMatrix[dst][0]);//velocity of the destination node between t0 and t1
}

if ((posMatrix[dst][8]-posMatrix[dst][4])==0)//to skip nan
{
  vel2_dst=0;
}
else
{
  vel2_dst=sqrt(pow((posMatrix[dst][9]-posMatrix[dst][5]),2)+pow((posMatrix[dst][10]-posMatrix[dst][6]),2)+pow((posMatrix[dst][11]-posMatrix[dst][7]),2))/(posMatrix[dst][8]-posMatrix[dst][4]);//velocity of the destination node between t1 and t2
}

if ((posMatrix[dst][8]-posMatrix[dst][0])==0)//to skip nan
{
  accel_dst=0;
}
else
{
  accel_dst=(vel2_dst-vel1_dst)/(posMatrix[dst][8]-posMatrix[dst][0]);//acceleration
}

  if(vel2_src==vel2_dst && vel2_src==0)//if both the src and dst are hovering
     {
       time=totalTime;
       flag=false;
     }

  //this parameters are to simplify the equations
  double a0=posMatrix[src][9] - posMatrix[dst][9];
  double a1=cos(alpha_src)*sin(theta_src);
  double a2=cos(alpha_dst)*sin(theta_dst);
  double a3=posMatrix[src][10] - posMatrix[dst][10] ;
  double a4=sin(alpha_src)*sin(theta_src);
  double a5=sin(alpha_dst)*sin(theta_dst) ;
  double a6=posMatrix[src][11] - posMatrix[dst][11]; 
  double a7=cos(theta_src);
  double a8=cos(theta_dst);
  //

//Bisection method to find the positive root
  double tOld=0;
  double tNew=totalTime;
  double t;

  double dis1=0;
  double dis2=0;
  double dis3=0;
  
  while(flag)
       {
         t=tOld;
         dis1 =sqrt(pow((a0 + a1*((accel_src*pow(t,2))/2 + vel2_src*t) - a2*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) +pow((a3 + a4*((accel_src*pow(t,2))/2 + vel2_src*t) - a5*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) +pow((a6 + a7*((accel_src*pow(t,2))/2 + vel2_src*t) - a8*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2))-TrRange;
         
         t=tNew;
         dis2 =sqrt(pow((a0 + a1*((accel_src*pow(t,2))/2 + vel2_src*t) - a2*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) +pow((a3 + a4*((accel_src*pow(t,2))/2 + vel2_src*t) - a5*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) + pow((a6 + a7*((accel_src*pow(t,2))/2 + vel2_src*t) - a8*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2))-TrRange;
         
         if(dis1*dis2==0)
           {
             flag=false;
             if(dis1==0)
               {
                 tNew=tOld;
               }
           }
         else if(dis1*dis2<0)
           {
             t=0.5*(tNew+tOld);
             dis3=sqrt(pow((a0 + a1*((accel_src*pow(t,2))/2 + vel2_src*t) - a2*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) +pow((a3 + a4*((accel_src*pow(t,2))/2 + vel2_src*t) - a5*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2) + pow((a6 + a7*((accel_src*pow(t,2))/2 + vel2_src*t) - a8*((accel_dst*pow(t,2))/2 + vel2_dst*t)),2))-TrRange;
             if(dis1*dis3==0)
               {
                 tNew=t;
                 flag=false;
               }
             else if(dis1*dis3<0)
               {
                     tNew=t;
               }
             else
               {
                 tOld=t;
               }
           }
         else//dis1*dis2>0
           {
             //"Bolzano's condition has not satisfied with the starting points."
             //There is no root in the interval or there is an even number of roots
             //due to the shape of the curve, it could not have an even number of roots
             //hence we consider the positive root beyond the interval [0 totalTime]
             // and consider it as the maximum time of the simulation which is totalTime   
             flag=false;
           }
    if (abs(tNew-tOld)<0.001)
        flag=false;
       }
//end of bisection method
  time=tNew;
  return time;
}

double 
distance(uint32_t src,uint32_t dst)
{
  double dist=0;
  dist=sqrt(pow((posMatrix[src][9]-posMatrix[dst][9]),2)+pow((posMatrix[src][10]-posMatrix[dst][10]),2)+pow((posMatrix[src][11]-posMatrix[dst][11]),2));//the distance of the third (last) position 
  return dist;
}

void 
setupMobilityTrack(NodeContainer adhocNodes)
{
  for (int i=0;i<nWifi;i++)
      {
         mobilityTrackingSocket[i] = Socket::CreateSocket (adhocNodes.Get (i), TcpSocketFactory::GetTypeId ());
         mobilityTrackingSocket[i]->Bind ();
      }
 traceUAVpositions();//to initilize the posMatrix
 traceUAVpositions();//
 traceUAVpositions();//
}

void 
updateLifetimes()
{
  double dis=0;
  for(int i=0;i<nWifi-1;i++)
     {
     for(int j=i+1;j<nWifi;j++)
         {
           dis=distance(i,j);
           if(dis>TrRange)
              {
                linkLifetime[i][j]=0;
                linkLifetime[j][i]=0;
              }
           else
              {
                linkLifetime[i][j]=lifeTime(i,j);
                linkLifetime[j][i]=linkLifetime[i][j];
              }
         }
     }
}


int 
Flow::posInRout(int intermediateSink)
{
  int pos=-1;
  for (int i=0; i<=nWifi;i++)
      {
         if (this->rout[i]==intermediateSink)
             pos=i;
      }
  return pos;
}

void 
Flow::CwndTracer (uint32_t oldval, uint32_t newval)
{
  NS_LOG_INFO ("Moving cwnd from " << oldval << " to " << newval);
}

void 
Flow::accept(Ptr<Socket> socket,const ns3::Address& from)
{ 
    std::cout<<"Accept for flow No. "<<this->flowNo<<std::endl;
    socket->SetRecvCallback (MakeCallback (&Flow::ReceivePacket,this));
}

void 
Flow::ReceivePacket (Ptr<Socket> socket)
 {
  Ptr<Packet> packet = socket->Recv ();
  if (socket->GetNode()->GetId()==this->sink)
     {      
       this->packetReceived=true;
       this->currentRxPackets+=1;
       this->currentRxBytes+=packet->GetSize ();
       if(this->currentRxBytes>=totalRxBytes && !(this->successfullyTerminated))
          {           
            this->sourceSocket[this->source]->Close();
            socket->ShutdownRecv();
            closeIntermediateSockets();
            this->successfullyTerminated=true;
            this->FCT=Simulator::Now ().GetSeconds ();
            this->throughput=(double(this->currentRxBytes)*8/this->FCT)/2800000;//the BW is 2.8Mbps 
            std::cout<<"Flow "<<this->flowNo<< ": currentTxBytes= "<<this->currentTxBytes<<",  currentRxBytes= "<<this->currentRxBytes<<",  Throuput= "<< this->throughput<<", FCT: "<<this->FCT<<std::endl;
          }
     }
  else if (this->successfullyTerminated)
    {
      std::cout<<"Flow no. "<<this->flowNo<<" shutdown socket No. "<<socket->GetNode()->GetId()<<std::endl;
      socket->ShutdownRecv();
    }
  else
    {
      int pos;
      std::stringstream ssPos;
      pos=posInRout(socket->GetNode()->GetId());
      if(pos==-1)
         {
           ssPos<<"10.0.0."<<this->sink+1;//just in case that the packet arrives after the rout change
           //std::cout<<"Flow no. "<<this->flowNo<<" position -1, send to "<<ssPos.str()<<std::endl;           
         }
      else
         {     
           ssPos<<"10.0.0."<<this->rout[pos+1]+1;
           std::string str=ssPos.str();
           const char * c = str.c_str();
           Ipv4Address dstAddr (c);

           packet->RemoveAllPacketTags ();
           packet->RemoveAllByteTags ();
           this->sourceSocket[socket->GetNode()->GetId()]->SendTo (packet, 0,  InetSocketAddress (dstAddr,port));
         }
    }
 }

void
Flow::WriteUntilBufferFull (Ptr<Socket> sourceSocket, uint32_t txSpace)
{ 
  while (this->currentRxBytes < totalRxBytes && sourceSocket->GetTxAvailable () > 0) 
    { 
      uint32_t left = totalRxBytes - this->currentRxBytes;
      uint32_t dataOffset = this->currentTxBytes % writeSize;
      uint32_t toWrite = writeSize - dataOffset;
      toWrite = std::min (toWrite, left);
      toWrite = std::min (toWrite, sourceSocket->GetTxAvailable ());
      int amountSent = sourceSocket->Send (&data[dataOffset], toWrite, 0);  
      if(amountSent < 0)
        {
          return;  
        }
      this->currentTxBytes += amountSent;
      this->currentTxPackets+=1;
    } 
  if (this->currentRxBytes>=totalRxBytes)
     {
        sourceSocket->Close();
     }
}

void
setupMobility(double X, double Y, double Z, NodeContainer adhocNodes,uint32_t mobilityModel,int64_t streamIndex)
{
NS_LOG_FUNCTION("setupMobility");

  ObjectFactory pos;
  std::stringstream sX;
  std::stringstream sY;
  std::stringstream sZ;
  sX<<"ns3::UniformRandomVariable[Min=0.0|Max="<<X<<"]";
  sY<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Y<<"]";
  sZ<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Z<<"]";
  
  pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
  pos.Set ("X", StringValue (sX.str()));
  pos.Set ("Y", StringValue (sY.str()));
  pos.Set ("Z", StringValue (sZ.str()));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  int nodeSpeed=20;  //in m/s
  int nodePause = 0; //in s
  double direction=6.283185307; // in radian
  double pitch=0.05; // in radian

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  std::stringstream ssDirection;
  ssDirection << "ns3::UniformRandomVariable[Min=0|Max=" << direction << "]";
  std::stringstream ssPitch;
  ssPitch << "ns3::UniformRandomVariable[Min="<< pitch <<"|Max=" << pitch << "]";
  std::stringstream ssNormVelocity;
  ssNormVelocity <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]";
  std::stringstream ssNormDirection;
  ssNormDirection <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]";
  std::stringstream ssNormPitch;
  ssNormPitch <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]";

  switch (mobilityModel)
    {
    case 1:
      mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
      break;
    case 2:
      mobilityAdhoc.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
                     "Bounds", BoxValue (Box (0, X, 0, Y, 0, Z)),
                     "TimeStep", TimeValue (Seconds (0.5)),
                     "Alpha", DoubleValue (0.85),
                     "MeanVelocity", StringValue (ssSpeed.str()),
                     "MeanDirection", StringValue (ssDirection.str()),
                     "MeanPitch", StringValue (ssPitch.str()),
                     "NormalVelocity", StringValue (ssNormVelocity.str()),
                     "NormalDirection", StringValue (ssNormDirection.str()),
                     "NormalPitch", StringValue (ssNormPitch.str()));
      break;
/*    case 3:
      mobilityAdhoc.SetMobilityModel ("ns3::SteadyStateRandomWaypointMobilityModel",
                                  //"Speed", StringValue (ssSpeed.str ()),
                                  //"Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAllocSSRWP));
      
      break;
    case 4:
      mobilityAdhoc.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                              "Mode", StringValue ("Time"),
                              "Time", StringValue ("2s"),
                              "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "Bounds", BoxValue (Box (0, X, 0, Y, 0, Z)));
*/
    default:
      NS_FATAL_ERROR ("No such model:" << mobilityModel);
    }
  


/*If you want to fix the node positions

mobilityAdhoc.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        Vector node1_Position(0.1, 0.1, 0.0);
	Vector node2_Position(50.0, 0.1,0.0);
	Vector node3_Position(100.0, 0.1, 0.0);	

	ListPositionAllocator myListPositionAllocator;
	myListPositionAllocator.Add(node1_Position);
	myListPositionAllocator.Add(node2_Position);
	myListPositionAllocator.Add(node3_Position);
	
	mobilityAdhoc.SetPositionAllocator(&myListPositionAllocator);
*/

  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);
  NS_UNUSED (streamIndex); // From this point, streamIndex is unused
}

YansWifiPhyHelper
setupWifiPhy(double txp)
{
  YansWifiPhyHelper wifiPhy; 
  wifiPhy =  YansWifiPhyHelper::Default ();
  
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));
  return wifiPhy;
}

Ipv4InterfaceContainer 
setupIP(NetDeviceContainer adhocDevices)
{
  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  return adhocInterfaces;
}

void 
Flow::findRout()
{  
  for(int i=0;i<nWifi;i++)
       this->rout[i]=-1;
  int path[nWifi]; 
  bool pathAvailable=true;
  double objective;
  objective=10000;//Just a big number
  double netGraph[nWifi][nWifi];
  for(int i=0;i<nWifi;i++)
    {
      for(int j=0;j<nWifi;j++)
         {
           netGraph[i][j]=linkLifetime[i][j];
         }
     }
  while(pathAvailable)
    { 
      for (int i=0; i<nWifi; i++)
         {
           path[i]=-1;
         }      
      BFS(netGraph,this->source,this->sink,path);
      if(path[0]==-1)
        {
          pathAvailable=false;
        }  
      else if (objectiveValue(path)<objective)
         {
           objective=objectiveValue(path);
           for(int i=0;i<nWifi;i++)
               this->rout[i]=path[i];
           removeLowestLifetimes(netGraph,path);
         }
      else
         {
           removeLowestLifetimes(netGraph,path);
         }      
    }
  if(this->rout[0]==-1)//there is no rout
     {
       this->pathLen=0;
      }  
  else
     {
       this->pathLen=routLen(this->rout);

       std::cout<<"Flow No. "<<this->flowNo<<": time "<<Simulator::Now ().GetSeconds ()<<": The path is: ";
       for(int i=0;i<=pathLen;i++)
           std::cout<<this->rout[i]<<" ";
       std::cout<<std::endl;

      } 
}

int 
routLen(int *path)
{
  int len=0;
  for (int i=0;i<nWifi;i++)
     if (path[i]>=0)
         len++;
  len=len-1;
  return len;
}

void 
Flow::setupConnection(Flow* m_flow,uint32_t m_source,uint32_t m_sink, uint16_t  port,Ipv4InterfaceContainer adhocInterfaces,NodeContainer adhocNodes)
{
  m_flow->source=m_source;
  m_flow->sink=m_sink;
  updateLifetimes();
  m_flow->findRout();
  if(m_flow->rout[0]==-1)
    {//there is no rout, now.
      Simulator::Schedule (Seconds (1.0), &Flow::setupConnection,m_flow,m_flow,m_flow->source,m_flow->sink,port,adhocInterfaces,adhocNodes);
    }
  else
    { 
      m_flow->setupSockets(m_flow);
      for(int i=0;i<pathLen;i++)
         {
           m_flow->sourceSocket[m_flow->rout[i]]->Connect (InetSocketAddress (adhocInterfaces.GetAddress (m_flow->rout[i+1]), port)); //connecting the source sockets
          }
      m_flow->sourceSocket[m_flow->rout[0]]->SetSendCallback (MakeCallback (&Flow::WriteUntilBufferFull,m_flow));
      WriteUntilBufferFull (m_flow->sourceSocket[m_flow->rout[0]], m_flow->sourceSocket[m_flow->rout[0]]->GetTxAvailable ());
                  // Trace changes to the congestion window
            Config::ConnectWithoutContext ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&Flow::CwndTracer,this)); 

      m_flow->packetReceived=true;
      m_flow->updatePathIfNotAlive(m_flow);
    } 
}


void
Flow::setupSockets(Flow* m_flow)
{std::cout<<"Setup socket for Flow No. "<<flowNo<<std::endl;
//Create and bind a src and a sink socket to each node...
      for (int i=0;i<nWifi;i++)
          {
            //Create and bind a sink socket...    
            m_flow->sinkSocket[i] = Socket::CreateSocket (adhocNodes.Get (i), TcpSocketFactory::GetTypeId ());  
            TypeId tid = TypeId::LookupByName ("ns3::TcpNewReno");
            Config::Set ("/NodeList/*/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tid));
            InetSocketAddress local= InetSocketAddress (Ipv4Address::GetAny (), port);
            m_flow->sinkSocket[i]->Bind(local);
            m_flow->sinkSocket[i]->Listen();
            m_flow->sinkSocket[i]->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&Flow::accept,m_flow));

            // Create and bind a source socket...
            m_flow->sourceSocket[i] = Socket::CreateSocket (adhocNodes.Get (i), TcpSocketFactory::GetTypeId ());
            m_flow->sourceSocket[i]->Bind ();
           }
}
