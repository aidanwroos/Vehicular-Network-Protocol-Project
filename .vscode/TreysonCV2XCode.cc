#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

// Global variables for metrics
uint32_t totalPacketsSent = 0;
uint32_t totalPacketsReceived = 0;
uint32_t totalCollisions = 0; // Track collisions
Time totalDelay = Seconds(0);

// Callbacks for metrics
void PacketSentCallback(Ptr<const Packet> packet) {
    totalPacketsSent++;
}

void PacketReceivedCallback(Ptr<const Packet> packet, const Address &addr) {
    totalPacketsReceived++;
    Time delay = Simulator::Now() - packet->GetUid();
    totalDelay += delay;
}

void PhyCollisionCallback(Ptr<const Packet> packet) {
    totalCollisions++;
}

int main(int argc, char *argv[]) {
    // Simulation parameters with default values
    uint32_t numVehicles = 20;    // Number of vehicles
    double vehicleSpeed = 20.0;   // Vehicle speed in m/s
    double vehicleDistance = 10.0; // Distance between vehicles in meters
    double simTime = 10.0;        // Simulation time in seconds

    // Command-line arguments
    CommandLine cmd;
    cmd.AddValue("numVehicles", "Number of vehicles in the simulation", numVehicles);
    cmd.AddValue("vehicleSpeed", "Vehicle speed in m/s", vehicleSpeed);
    cmd.AddValue("vehicleDistance", "Distance between vehicles in meters", vehicleDistance);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.Parse(argc, argv);

    // Create nodes
    NodeContainer vehicles;
    vehicles.Create(numVehicles);

    // Mobility model
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(vehicles);
    for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
        Ptr<ConstantVelocityMobilityModel> mobilityModel = vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        mobilityModel->SetPosition(Vector(i * vehicleDistance, 0.0, 0.0));
        mobilityModel->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));
    }

    // C-V2X configuration using LTE sidelink
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    NetDeviceContainer devices = lteHelper->InstallUeDevice(vehicles);
    lteHelper->EnableSidelink();

    // Install the Internet stack
    InternetStackHelper internet;
    internet.Install(vehicles);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Applications
    uint16_t port = 8080;
    OnOffHelper onOff("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("255.255.255.255"), port)); // Broadcast
    onOff.SetAttribute("DataRate", StringValue("6Mbps"));
    onOff.SetAttribute("PacketSize", UintegerValue(1000));
    onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer apps = onOff.Install(vehicles);
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(simTime));

    // Trace callbacks
    devices.Get(0)->TraceConnectWithoutContext("PhyTxEnd", MakeCallback(&PacketSentCallback));
    devices.Get(0)->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&PacketReceivedCallback));
    devices.Get(0)->TraceConnectWithoutContext("PhyTxDrop", MakeCallback(&PhyCollisionCallback));

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Metrics calculation
    double avgDelay = totalPacketsReceived > 0 ? totalDelay.GetSeconds() / totalPacketsReceived : 0;
    double PDR = totalPacketsSent > 0 ? (double)totalPacketsReceived / totalPacketsSent * 100 : 0;
    double collisionRate = totalPacketsSent > 0 ? (double)totalCollisions / totalPacketsSent * 100 : 0;

    // Print results
    std::cout << "Number of Vehicles: " << numVehicles << std::endl;
    std::cout << "Vehicle Speed: " << vehicleSpeed << " m/s" << std::endl;
    std::cout << "Vehicle Distance: " << vehicleDistance << " meters" << std::endl;
    std::cout << "Total Packets Sent: " << totalPacketsSent << std::endl;
    std::cout << "Total Packets Received: " << totalPacketsReceived << std::endl;
    std::cout << "Packet Delivery Ratio (PDR): " << PDR << "%" << std::endl;
    std::cout << "Average Delay: " << avgDelay << " seconds" << std::endl;
    std::cout << "Collision Rate: " << collisionRate << "%" << std::endl;

    Simulator::Destroy();
    return 0;
}

