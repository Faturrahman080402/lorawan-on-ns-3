#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MeshTopologyExample");

int main(int argc, char *argv[])
{
    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer clientNodes;
    clientNodes.Create(4);

    NodeContainer gatewayNodes;
    gatewayNodes.Create(2);

    Ptr serverNode = CreateObject();

    NodeContainer allNodes;
    allNodes.Add(clientNodes);
    allNodes.Add(gatewayNodes);
    allNodes.Add(serverNode);

    // Configure Point-to-Point links
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("200MB/s"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    InternetStackHelper stack;
    stack.Install(allNodes);

    Ipv4AddressHelper address;
    char baseAddr[16];
    std::vector interfaces;

    // Connect each client to all gateways
    for (uint32_t i = 0; i < clientNodes.GetN(); ++i)
    {
        for (uint32_t j = 0; j < gatewayNodes.GetN(); ++j)
        {
            NodeContainer linkNodes(clientNodes.Get(i), gatewayNodes.Get(j));
            NetDeviceContainer linkDevices = p2p.Install(linkNodes);

            sprintf(baseAddr, "192.0.%d.%d", i + 1, j + 1);
            address.SetBase(baseAddr, "255.255.255.0");
            Ipv4InterfaceContainer iface = address.Assign(linkDevices);
            interfaces.push_back(iface);
        }
    }

    // Connect Client 2 directly to the server
    NodeContainer client2Server(clientNodes.Get(1), serverNode);
    NetDeviceContainer client2ServerDevices = p2p.Install(client2Server);

    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(client2ServerDevices);

    // Connect each gateway to the server
    for (uint32_t i = 0; i < gatewayNodes.GetN(); ++i)
    {
        NodeContainer linkNodes(gatewayNodes.Get(i), serverNode);
        NetDeviceContainer linkDevices = p2p.Install(linkNodes);

        sprintf(baseAddr, "172.16.%d.0", i + 1);
        address.SetBase(baseAddr, "255.255.255.0");
        address.Assign(linkDevices);
    }

    // UDP Echo Server on the server node
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApp = echoServer.Install(serverNode);
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(20.0));

    // UDP Echo Clients on each client node
    for (uint32_t i = 0; i < clientNodes.GetN(); ++i)
    {
        for (uint32_t j = 0; j < gatewayNodes.GetN(); ++j)
        {
            UdpEchoClientHelper echoClient(interfaces[i * gatewayNodes.GetN() + j].GetAddress(1), 9); // Gateway address
            echoClient.SetAttribute("MaxPackets", UintegerValue(5));
            echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
            echoClient.SetAttribute("PacketSize", UintegerValue(1024));

            ApplicationContainer clientApp = echoClient.Install(clientNodes.Get(i));
            clientApp.Start(Seconds(2.0 + i));
            clientApp.Stop(Seconds(15.0));
        }
    }

    // Populate Routing Tables
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Mobility setup
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(allNodes);

    Ptr mobilityModel;

    // Set positions for clients
    mobilityModel = clientNodes.Get(0)->GetObject();
    mobilityModel->SetPosition(Vector(0.0, 100.0, 0.0));

    mobilityModel = clientNodes.Get(1)->GetObject();
    mobilityModel->SetPosition(Vector(100.0, 200.0, 0.0));

    mobilityModel = clientNodes.Get(2)->GetObject();
    mobilityModel->SetPosition(Vector(100.0, 0.0, 0.0));

    mobilityModel = clientNodes.Get(3)->GetObject();
    mobilityModel->SetPosition(Vector(25.0, 200.0, 0.0));

    // Set positions for gateways
    mobilityModel = gatewayNodes.Get(0)->GetObject();
    mobilityModel->SetPosition(Vector(200.0, 0.0, 0.0)); // Gateway 1

    mobilityModel = gatewayNodes.Get(1)->GetObject();
    mobilityModel->SetPosition(Vector(200.0, 100.0, 0.0)); // Gateway 2

    // Set position for server
    mobilityModel = serverNode->GetObject();
    mobilityModel->SetPosition(Vector(300.0, 150.0, 0.0));

    // Animation setup
    AnimationInterface anim("1.xml");

    for (uint32_t i = 0; i < clientNodes.GetN(); ++i)
    {
        anim.UpdateNodeSize(clientNodes.Get(i)->GetId(), 5.0, 5.0);
        anim.UpdateNodeColor(clientNodes.Get(i), 0, 0, 255);
        anim.UpdateNodeDescription(clientNodes.Get(i), "Client " + std::to_string(i + 1));
    }

    for (uint32_t i = 0; i < gatewayNodes.GetN(); ++i)
    {
        anim.UpdateNodeSize(gatewayNodes.Get(i)->GetId(), 7.0, 7.0);
        anim.UpdateNodeColor(gatewayNodes.Get(i), 255, 0, 0);
        anim.UpdateNodeDescription(gatewayNodes.Get(i), "Gateway " + std::to_string(i + 1));
    }

    anim.UpdateNodeSize(serverNode->GetId(), 7.0, 7.0);
    anim.UpdateNodeColor(serverNode, 0, 255, 0);
    anim.UpdateNodeDescription(serverNode, "Server");

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
