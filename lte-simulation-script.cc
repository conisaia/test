#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/lte-module.h"
#include "ns3/lte-handover-algorithm.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/constant-velocity-helper.h"

using namespace ns3;

//NS_LOG_COMPONENT_DEFINE ("LTESimulationScript");

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}


/**
 *
 * simulation script for a X2-based handover.
 * It instantiates eNodeBs (C_ and M_ Antennas), attaches UEs to each eNB and
 * triggers handovers
 *
 * Simulation runs for 50.0 seconds. 
 * Simulation parameters are adjustable (simulation time, data rate, max packets...)
 *
 */
int
main (int argc, char *argv[])
{
  uint16_t numberOfUes = 94;   // number of user nodes
  uint16_t numberOfEnbs = 12;   // Number of antennas
  uint16_t numBearersPerUe = 2;
  double simTime = 100.0;
  std::string traceFile = "/home/pisaia/ns-allinone-3.29/ns-3.29/scratch/ns2mobility.tcl"
  //double distance = 5.0;

  // change some default attributes so that they are reasonable for
  // this scenario, but do this before processing command line
  // arguments, so that the user is allowed to override these settings

  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (10000)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (100));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numberOfUes", "Number of UEs", numberOfUes);
  cmd.AddValue ("numberOfEnbs", "Number of eNodeBs", numberOfEnbs);
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.Parse (argc, argv);


  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  //lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
  lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
  //lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");


  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.01)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);


  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  
  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  

  //Install Mobility Model for ENBs (Antennas)
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  
  positionAlloc->Add (Vector (262.66, -852.78, 0));
  positionAlloc->Add (Vector (529.37, -515.46, 0));
  positionAlloc->Add (Vector (666.65, -1123.42, 0));
  positionAlloc->Add (Vector (1066.73, -770.41, 0));
  positionAlloc->Add (Vector (1235.38, -1598.02, 0));
  positionAlloc->Add (Vector (1509.94, -1472.50, 0));
  positionAlloc->Add (Vector (1906.10, -2045.16, 0));
  positionAlloc->Add (Vector (2867.06, -1856.89, 0));
  positionAlloc->Add (Vector (297.96, -919.46, 0));
  positionAlloc->Add (Vector (1266.76, -1060.66, 0));
  positionAlloc->Add (Vector (1509.94, -1472.50, 0));
  positionAlloc->Add (Vector (2282.64, -2237.35, 0));
  
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (enbNodes);
	
  //Install Mobility Helper for UEs (Users-Cars)
  Ns2MobilityHelper ueMobility = Ns2MobilityHelper (traceFile);
  //ueMobility.SetMobilityModel ("ns3::Ns2MobilityHelper");
  NodeContainer ueNodes;
  ueNodes.Create (numberOfUes);
  ueMobility.Install (ueNodes.Begin(), ueNodes.End()); // configure movements for each node
  /*
  for (int i = 0; i < numberOfUes; i++)
    {
      lte.AddDownlinkChannelRealization (mobility, ueNodes.Get(i)->GetObject<Mobility>(), ue[i]->GetPhy ());
	}
  
  */
  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  // Attach first 0-3 UEs to C1
  for (uint16_t i = 0; i < numberOfUes; i++)
    {
      lteHelper->Attach (ueLteDevs.Get (i), enbLteDevs.Get (0));
    }

  NS_LOG_LOGIC ("setting up applications");

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (1.0));

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
          ++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          clientApps.Add (dlClientHelper.Install (remoteHost));
          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          serverApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort;
          dlpf.localPortEnd = dlPort;
          tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }


  // Add X2 interface
  lteHelper->AddX2Interface (enbNodes);

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));


  // connect custom trace sinks for RRC connection establishment and handover notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));

////////// Net Animator //////////

  // Net animator is used to view the topology and simulation traffic
  AnimationInterface anim ("lte-simulation-script.xml");
  anim.SetBackgroundImage("/home/pisaia/ns-allinone-3.29/ns-3.29/scratch/test3.png", 0, 0, 0, 0, 0);
  

  // uncomment the following line to enable meta deta
  //anim.EnablePacketMetadata (true);
  //anim.SetMobilityPollInterval (Seconds (1));

  // updating Antennas name for net animator
  anim.UpdateNodeDescription (0, "PGW");
  anim.UpdateNodeDescription (1, "remoteHost");
  anim.UpdateNodeDescription (2, "C1");
  anim.UpdateNodeDescription (3, "C2");
  anim.UpdateNodeDescription (4, "C3");
  anim.UpdateNodeDescription (5, "C4");
  anim.UpdateNodeDescription (6, "C5");
  anim.UpdateNodeDescription (7, "C6");
  anim.UpdateNodeDescription (8, "C7");
  anim.UpdateNodeDescription (9, "C8");
  anim.UpdateNodeDescription (10, "M1");
  anim.UpdateNodeDescription (11, "M2");
  anim.UpdateNodeDescription (12, "M3");
  anim.UpdateNodeDescription (13, "M4");
  anim.UpdateNodeDescription (14, "U1"); // handover user
  anim.UpdateNodeDescription (15, "U2"); // handover user
  anim.UpdateNodeDescription (16, "U3"); // handover user
  anim.UpdateNodeDescription (17, "U4"); // handover user
  anim.UpdateNodeDescription (18, "U5"); // handover user
  /*
  anim.UpdateNodeDescription (5, "C4");
  anim.UpdateNodeDescription (6, "C5");
  anim.UpdateNodeDescription (7, "C6");
  anim.UpdateNodeDescription (8, "C7");
  anim.UpdateNodeDescription (9, "C8");
  anim.UpdateNodeDescription (10, "C9");
  anim.UpdateNodeDescription (11, "C10");
  anim.UpdateNodeDescription (12, "C11");
  anim.UpdateNodeDescription (13, "M1");
  anim.UpdateNodeDescription (14, "M2");
  anim.UpdateNodeDescription (15, "M3");
  anim.UpdateNodeDescription (16, "M4");
  anim.UpdateNodeDescription (17, "M5");
  anim.UpdateNodeDescription (18, "M6");
  anim.UpdateNodeDescription (19, "M7");
  anim.UpdateNodeDescription (20, "M8");
  anim.UpdateNodeDescription (21, "M9");
  anim.UpdateNodeDescription (22, "HOUser"); // hand over user
  */

  // update node sizes and colors
  int nodes = numberOfUes + numberOfEnbs + 2;

  // PGW is blue
  anim.UpdateNodeSize (0, 5, 5);
  anim.UpdateNodeColor (0, 0, 0, 255); // Optional

  // remoteHost is blue
  anim.UpdateNodeSize (1, 3, 3);
  anim.UpdateNodeColor (1, 0, 0, 255); // Optional

  // antennas are red
  for (int i =2; i< 7 + 2 ; i++)
     {
       anim.UpdateNodeSize (i, 5, 5);
       anim.UpdateNodeColor (i, 255, 0, 0); // Optional
     }
   
   // users are green
  for (int i =numberOfUes + 4; i< nodes ; i++)
    {
      anim.UpdateNodeSize (i, 3, 3);
      anim.UpdateNodeColor (i, 0, 255, 0); // Optional
    }

//////////////Start Simulation


  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  // GtkConfigStore config;
  // config.ConfigureAttributes ();

  Simulator::Destroy ();
  return 0;

}
