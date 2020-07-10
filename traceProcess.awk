BEGIN{
       aodvBytes=0
       aodvPackets=0
       tmax=75.695

}

{
   if($2<tmax && $50=="ns3::aodv::TypeHeader"){
	   aodvBytes=aodvBytes+$40+$46+4
	   aodvPackets++
	}
   if($2<tmax && $49=="ns3::aodv::TypeHeader"){
	   aodvBytes=aodvBytes+$39+$45+4
	   aodvPackets++
	}
}

END{
     print "AODV traffic (Bytes)    "  aodvBytes
     print "AODV traffic (pakcets)  "  aodvPackets
}
