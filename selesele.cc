Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
loss->SetPathLossExponent(3.76);
loss->SetReference(1, 7.7);

Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
Ptr<Channel> channel = CreateObject<PropagationLossPropagationDelayModel>(loss, delay);

// Membuat LoraHelper
LoraHelper loraHelper;
LoraPhyHelper phyHelper = LoraPhyHelper();
LorawanMacHelper macHelper = LorawanMacHelper();

// Menghubungkan channel ke PHY helper
phyHelper.SetChannel(channel);

// Atur MAC LoRa sebagai End Device
macHelper.SetDeviceType(LorawanMacHelper::ED);

// Instal PHY dan MAC pada end devices
loraHelper.Install(phyHelper, macHelper, endDevices);

// Instal network server
NetworkServerHelper networkServerHelper;
networkServerHelper.Install(networkServer);

// Instal forwarder pada gateway
ForwarderHelper forwarderHelper;
forwarderHelper.Install(gateways);

// Tambahkan aplikasi pada end devices
PeriodicalSenderHelper appHelper = PeriodicalSenderHelper();
appHelper.SetPeriod(Seconds(10));  // Perangkat mengirim data setiap 10 detik
appHelper.Install(endDevices);

// Jalankan simulasi
Simulator::Run();
Simulator::Destroy();
