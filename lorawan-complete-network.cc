#include "ns3/building-allocator.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/buildings-helper.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("ComplexLorawanNetworkExample");

int nDevices = 4;
int nGateways = 2;
double radiusMeters = 6400;
double simulationTimeSeconds = 600;
bool realisticChannelModel = false;
int appPeriodSeconds = 600;
bool printBuildingInfo = true;

int main(int argc, char *argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("radius", "The radius (m) of the area to simulate", radiusMeters);
    cmd.AddValue("realisticChannel", "Whether to use a more realistic channel model", realisticChannelModel);
    cmd.AddValue("simulationTime", "The time (s) for which to simulate", simulationTimeSeconds);
    cmd.AddValue("appPeriod", "The period in seconds to be used by periodically transmitting applications", appPeriodSeconds);
    cmd.AddValue("print", "Whether or not to print building information", printBuildingInfo);
    cmd.Parse(argc, argv);

    // Set up logging
    LogComponentEnable("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

    LogComponentEnable("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);

    Time appPeriod = Seconds(appPeriodSeconds);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho",
                                  DoubleValue(radiusMeters),
                                  "X",
                                  DoubleValue(0.0),
                                  "Y",
                                  DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    if (realisticChannelModel)
    {
        Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel>();
        loss->SetNext(shadowing);

        Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();
        shadowing->SetNext(buildingLoss);
    }

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    LorawanMacHelper macHelper = LorawanMacHelper();
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    NetworkServerHelper nsHelper = NetworkServerHelper();
    ForwarderHelper forHelper = ForwarderHelper();

    NodeContainer endDevices;
    endDevices.Create(nDevices);

    mobility.Install(endDevices);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    macHelper.SetRegion(LorawanMacHelper::EU);

    helper.Install(phyHelper, macHelper, endDevices);

    NodeContainer gateways;
    gateways.Create(nGateways);

    mobility.Install(gateways);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);

    helper.Install(phyHelper, macHelper, gateways);

    BuildingsHelper::Install(endDevices);
    BuildingsHelper::Install(gateways);

    if (printBuildingInfo)
    {
        BuildingsHelper::PrintBuildings();
    }

    for (NodeContainer::Iterator it = gateways.Begin(); it != gateways.End(); ++it)
    {
        Ptr<ConstantPositionMobilityModel> mobility = (*it)->GetObject<ConstantPositionMobilityModel>();
        Vector position = mobility->GetPosition();
        NS_LOG_INFO("Gateway positioned at: " << position);
    }

    for (NodeContainer::Iterator it = endDevices.Begin(); it != endDevices.End(); ++it)
    {
        Ptr<ConstantPositionMobilityModel> mobility = (*it)->GetObject<ConstantPositionMobilityModel>();
        Vector position = mobility->GetPosition();
        NS_LOG_INFO("End device positioned at: " << position);
    }

    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(appPeriod);

    appHelper.Install(endDevices);

    nsHelper.Install(gateways);
    forHelper.Install(gateways);

    Simulator::Stop(Seconds(simulationTimeSeconds));

    std::time_t result = std::time(nullptr);
    std::string time = std::asctime(std::localtime(&result));
    time.pop_back();

    std::string filename = "output/simulation-results-" + time + ".csv";
    std::replace(filename.begin(), filename.end(), ' ', '_');
    std::replace(filename.begin(), filename.end(), ':', '-');

    std::ofstream file;
    file.open(filename);

    if (!file.is_open())
    {
        NS_LOG_ERROR("Unable to open file for writing results");
    }
    else
    {
        NS_LOG_INFO("Saving simulation results to " << filename);
    }

    Ptr<PacketTrackingHelper> packetTracker = helper.GetPacketTracker();

    Simulator::Run();

    NS_LOG_INFO("Simulation finished");

    if (file.is_open())
    {
        file << "DeviceID,PacketID,Timestamp" << std::endl;

        for (uint32_t i = 0; i < packetTracker->GetPacketCount(); ++i)
        {
            PacketTrackingHelper::PacketRecord record = packetTracker->GetPacketRecord(i);

            for (PacketTrackingHelper::NodePacketList::const_iterator it = record.txList.begin(); it != record.txList.end(); ++it)
            {
                file << it->nodeId << "," << i << "," << it->timestamp.GetSeconds() << std::endl;
            }
        }

        file.close();
        NS_LOG_INFO("Results saved to " << filename);
    }

    Simulator::Destroy();

    return 0;
}
