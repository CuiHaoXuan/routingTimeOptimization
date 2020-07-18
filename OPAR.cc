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

NS_LOG_COMPONENT_DEFINE ("OPAR");

void setupMobility(double, double,double,NodeContainer,uint32_t,int64_t);
YansWifiPhyHelper setupWifiPhy(double);
Ipv4InterfaceContainer setupIP(NetDeviceContainer);

static const uint16_t port = 50000;
static const uint32_t totalRxBytes = 5000;
static Ipv4InterfaceContainer adhocInterfaces;
static NetDeviceContainer adhocDevices;
static NodeContainer adhocNodes;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1040;
uint8_t data[writeSize];
static MobilityHelper mobilityAdhoc;

class Flow
{
public:
  Flow();
  void setupConnection(uint16_t,Ipv4InterfaceContainer,NodeContainer);

  double throughput;
  double FCT;//flow completion time
  bool successfullyTerminated;
  uint32_t source;
  uint32_t sink;

private:
  uint32_t currentTxBytes;
  uint32_t currentTxPackets;
  uint32_t currentRxBytes;
  uint32_t currentRxPackets;
  int pathLen;
  int *rout;
  
  Ptr<Socket> mainSourceSocket;
  Ptr<Socket> mainSinkSocket;
  Ptr<Socket> sourceSocket[100];
  Ptr<Socket> sinkSocket[100];

  void WriteUntilBufferFull (Ptr<Socket>, uint32_t);
  void accept(Ptr<Socket>,const ns3::Address&);
  void ReceivePacket (Ptr<Socket>);
  void CwndTracer (uint32_t , uint32_t );
  int * findRout(int,int);
  int routLen(int *);
  int posInRout(int );
};

Flow::Flow()
  : throughput(0),
    FCT(0),
    successfullyTerminated(false),
    currentTxBytes(0),
    currentTxPackets(0),
    currentRxBytes(0),
    currentRxPackets(0),
    pathLen(0)
{
}

//###################### main() ###########################
int main (int argc, char *argv[])
{
  int nSinks=1;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  double totalTime=100;
  uint32_t mobilityModel=1;//1-RWP, 2-GaussMarkov 
  uint32_t routingProtocol=2;//1-OLSR, 2-AODV, 3-DSDV, 4-DSR
  int nWifi=5;
  double txp=7.5;
  double X=100.0;
  double Y=10.0;
  double Z=50.0;
  
  CommandLine cmd;
  cmd.AddValue("nSinks", "No. of sinks to echo", nSinks);
  cmd.AddValue("streamIndex", "Stream Index to echo", streamIndex);
  cmd.AddValue("totalTime", "Total time to echo", totalTime);
  cmd.AddValue("mobilityModel", "Mobility model to echo (1-RWP, 2-GaussMarkov): ", mobilityModel);
  cmd.AddValue("routingProtocol", "Routing protocol to echo (1-OLSR, 2-AODV, 3-DSDV, 4-DSR): ", routingProtocol);
  cmd.AddValue("nWifi","No. of UAVs to echo",nWifi);
  cmd.AddValue("txp","Transmition power to echo",txp);
  cmd.AddValue("X","Area width to echo",X);
  cmd.AddValue("Y","Area Length to echo",Y);
  cmd.AddValue("Z","Area height to echo",Z);
  cmd.Parse (argc, argv);

  std::string phyMode ("DsssRate11Mbps");
  const std::string rate="2048bps";

  std::stringstream ss;
  ss<<"traceFiles/OPAR_UAV"<<nWifi<<"Con"<<nSinks;
  std::string tr_name (ss.str ());

  std::string routingName;
  std::string mobilityName;

switch (routingProtocol)
    {
    case 1:
      routingName = "OLSR";
      break;
    case 2:
      routingName = "AODV";
      break;
    case 3:
      routingName = "DSDV";
      break;
    case 4:
      routingName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << routingProtocol);
    }

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
  //setup routing protocol
  //setupRoutingProtocol(routingProtocol,adhocNodes);
  InternetStackHelper internet;
  internet.Install (adhocNodes);
  NS_LOG_UNCOND ("Setting up routing protocol...");
  // Later, we add IP addresses.
  adhocInterfaces=setupIP(adhocDevices);
  NS_LOG_UNCOND ("Setting up IP address for nodes...");
  
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
  
  for (int i = 0; i < nSinks; i++)
     {  
        UAVflow[i].source=i*2;
        UAVflow[i].sink=i*2+1;      
        UAVflow[i].setupConnection(port,adhocInterfaces,adhocNodes);
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
              std::cout<<"Flow no. "<<i+1<<":  Throughput= "<<UAVflow[i].throughput<<",  FCT= "<<UAVflow[i].FCT<<std::endl;
            }
      }
  std::cout<<count<< " flows out of total "<<nSinks<<" flows completed successfully."<<std::endl;
  std::cout<<"Average throughput: "<<throughput/count<<", Average FCT: "<<FCT/count<<std::endl; 
  Simulator::Destroy ();
return count;
  
}
//###################### end of main() ###########################


int 
Flow::posInRout(int intermediateSink)
{
  int pos=0;
  for (int i=0; i<pathLen;i++)
      {
         if (rout[i]==intermediateSink)
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
    socket->SetRecvCallback (MakeCallback (&Flow::ReceivePacket,this));
}

void 
Flow::ReceivePacket (Ptr<Socket> socket)
 {
/*
   Ptr<MobilityModel> mobility = socket->GetNode()->GetObject<MobilityModel>();
   Vector currentPos=mobility->GetPosition();
   std::cout<<"Current position: X= "<<currentPos.x<<", Y= "<<currentPos.y<<", Z= "<<currentPos.z <<std::endl;
  */ 

  Ptr<Packet> packet = socket->Recv ();

//#####################################################
if (socket->GetNode()->GetId()==sink)
   {
      std::cout<<"current received packets: "<<currentRxPackets<<std::endl;
      this->currentRxPackets+=1;
      this->currentRxBytes+=packet->GetSize ();
      if(currentRxBytes>=totalRxBytes)
         {   
            mainSourceSocket->Close ();
            for (int i=pathLen-3;i>=0;i--)
                {
                   sinkSocket[i]->ShutdownRecv();
                   sourceSocket[i]->Close();
                }
            socket->ShutdownRecv();
            mainSourceSocket->Close();
            this->successfullyTerminated=true;
            this->FCT=Simulator::Now ().GetSeconds ();
            this->throughput=(double(this->currentRxBytes)*8/this->FCT)/2800000;//the BW is 2.8Mbps 
            std::cout<<"currentTxBytes= "<<this->currentTxBytes<<",  currentRxBytes= "<<this->currentRxBytes<<",  Throuput= "<< this->throughput<<std::endl;
         }
    }
else if (successfullyTerminated)
    {
      socket->ShutdownRecv();
    }
else
    {
      int pos;
      pos=posInRout(socket->GetNode()->GetId());

      std::stringstream ss;
      ss<<"10.0.0."<<rout[pos]+1;
      std::string str=ss.str();
      const char * c = str.c_str();
      Ipv4Address dstAddr (c);

      packet->RemoveAllPacketTags ();
      packet->RemoveAllByteTags ();

      std::cout<<"Pos: "<<pos<<", des address: "<<dstAddr<<std::endl;
      sourceSocket[pos-1]->SendTo (packet, 0,  InetSocketAddress (dstAddr,port));
    }
//#####################################################
   SocketIpTosTag tosTag;
   if (packet->RemovePacketTag (tosTag))
     {
       NS_LOG_INFO (" TOS = " << (uint32_t)tosTag.GetTos ());
     }
   SocketIpTtlTag ttlTag;
   if (packet->RemovePacketTag (ttlTag))
     {
       NS_LOG_INFO (" TTL = " << (uint32_t)ttlTag.GetTtl ());
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
        sourceSocket->Close ();
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

int * 
Flow::findRout(int source,int sink)
{
static int rout[100];

for (int i=0; i<100; i++)
         rout[i]=-1;

rout[0]=source;//0
rout[1]=2;
rout[2]=3;
rout[3]=sink;//1
 
return rout;
}

int 
Flow::routLen(int *rout)
{
int len=0;
for (int i=0;i<100;i++)
     if (rout[i]>=0)
         len++;

return len;
}

void 
Flow::setupConnection(uint16_t  port,Ipv4InterfaceContainer adhocInterfaces,NodeContainer adhocNodes)
{
  //find the rout 
  rout=findRout(this->source,this->sink);
  pathLen=routLen(rout);
    
  // Create and bind the main sink socket...
  mainSinkSocket = Socket::CreateSocket (adhocNodes.Get (rout[pathLen-1]), TcpSocketFactory::GetTypeId ());   
 
  TypeId tid = TypeId::LookupByName ("ns3::TcpNewReno");
  Config::Set ("/NodeList/*/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tid));
  InetSocketAddress local= InetSocketAddress (Ipv4Address::GetAny (), port);

  mainSinkSocket->Bind(local);
  mainSinkSocket->Listen();
  
  mainSinkSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&Flow::accept,this));

// Create and bind the intermediate sockets...

  for (int i=pathLen-3;i>=0;i--)
       {
         // Create a source to send packets.  Instead of a full Application
         // and the helper APIs you might see in other example files, this example
         // will use sockets directly and register some socket callbacks as a sending
         // "Application".

         // Create and bind the source socket...
         //Ptr<Socket> sourceSocket;
         sourceSocket[i] = Socket::CreateSocket (adhocNodes.Get (rout[i+1]), TcpSocketFactory::GetTypeId ());
         sourceSocket[i]->Bind ();  
       //begin implementation of sending "Application"
         NS_LOG_LOGIC ("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
         sourceSocket[i]->Connect (InetSocketAddress (adhocInterfaces.GetAddress (rout[i+2]), port)); //connect
  
         // tell the tcp implementation to call WriteUntilBufferFull again
         // if we blocked and new tx buffer space becomes available
         //sourceSocket[i]->SetSendCallback (MakeCallback (&Flow::WriteUntilBufferFull,this));
         //WriteUntilBufferFull (sourceSocket[i], sourceSocket[i]->GetTxAvailable ());    


         sinkSocket[i] = Socket::CreateSocket (adhocNodes.Get (rout[i+1]), TcpSocketFactory::GetTypeId ()); 
 
         TypeId tid = TypeId::LookupByName ("ns3::TcpNewReno");
         Config::Set ("/NodeList/*/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tid));
         InetSocketAddress local= InetSocketAddress (Ipv4Address::GetAny (), port);

         sinkSocket[i]->Bind(local);
         sinkSocket[i]->Listen();

         sinkSocket[i]->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&Flow::accept,this));
       }

// Create and bind the main source socket...
  mainSourceSocket = Socket::CreateSocket (adhocNodes.Get (rout[0]), TcpSocketFactory::GetTypeId ()); 
  mainSourceSocket->Bind ();
  mainSourceSocket->Connect (InetSocketAddress (adhocInterfaces.GetAddress (rout[1]), port)); //connecting main source socket
  mainSourceSocket->SetSendCallback (MakeCallback (&Flow::WriteUntilBufferFull,this));
  WriteUntilBufferFull (mainSourceSocket, mainSourceSocket->GetTxAvailable ()); 

  // Trace changes to the congestion window
  Config::ConnectWithoutContext ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&Flow::CwndTracer,this));
}
