<Qucs Schematic 0.0.18>
<Properties>
  <View=0,-191,1190,926,0.826446,0,66>
  <Grid=10,10,1>
  <DataSet=UPSat_PA_RFOut_lumped_NoTLines_SDefaultvalues.dat>
  <DataDisplay=UPSat_PA_RFOut_lumped_NoTLines_SDefaultvalues.dpl>
  <OpenDisplay=1>
  <Script=UPSat_PA_RFOut_lumped_NoTLines_SDefaultvalues.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
</Symbol>
<Components>
  <C C4 1 510 260 17 -26 0 1 "22pF" 1 "" 0 "neutral" 0>
  <L L1 1 420 180 -26 10 0 0 "2.7nH" 1 "" 0>
  <GND * 1 510 300 0 0 0 0>
  <GND * 1 770 300 0 0 0 0>
  <Pac P1 1 1020 260 18 -26 0 1 "1" 1 "50 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
  <GND * 1 1020 290 0 0 0 0>
  <L L2 1 30 300 10 -26 0 1 "0.00055 uH" 1 "" 0>
  <R R2 1 30 360 15 -26 0 1 "2.6 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <GND * 1 30 390 0 0 0 0>
  <DCFeed L3 1 220 230 20 -26 0 1 "1 uH" 0>
  <GND * 1 310 310 0 0 0 0>
  <C C5 1 260 310 -26 17 0 0 "47 pF" 1 "" 0 "neutral" 0>
  <GND * 1 310 390 0 0 0 0>
  <C C6 1 260 390 -26 17 0 0 "1000 pF" 1 "" 0 "neutral" 0>
  <Eqn Eqn1 1 370 600 -28 15 0 0 "dBS11=dB(S[1,1])" 1 "yes" 0>
  <.SP SP1 1 180 600 0 78 0 0 "log" 1 "100MHz" 1 "2GHz" 1 "200" 1 "no" 0 "1" 0 "2" 0 "no" 0 "no" 0>
  <GND * 1 310 470 0 0 0 0>
  <C C7 1 260 470 -26 17 0 0 "3.3 uF" 1 "" 0 "neutral" 0>
  <C C2 1 770 260 17 -26 0 1 "2pF" 1 "" 0 "neutral" 0>
  <C C3 1 840 180 -26 17 0 0 "56pF" 1 "" 0 "neutral" 0>
</Components>
<Wires>
  <510 180 770 180 "" 0 0 0 "">
  <510 180 510 230 "" 0 0 0 "">
  <450 180 510 180 "" 0 0 0 "">
  <770 180 810 180 "" 0 0 0 "">
  <870 180 1020 180 "" 0 0 0 "">
  <510 290 510 300 "" 0 0 0 "">
  <1020 180 1020 230 "" 0 0 0 "">
  <30 180 220 180 "" 0 0 0 "">
  <30 180 30 270 "" 0 0 0 "">
  <220 180 390 180 "" 0 0 0 "">
  <220 180 220 200 "" 0 0 0 "">
  <220 260 220 310 "" 0 0 0 "">
  <220 310 230 310 "" 0 0 0 "">
  <290 310 310 310 "" 0 0 0 "">
  <220 310 220 390 "" 0 0 0 "">
  <220 390 230 390 "" 0 0 0 "">
  <290 390 310 390 "" 0 0 0 "">
  <220 390 220 470 "" 0 0 0 "">
  <220 470 230 470 "" 0 0 0 "">
  <290 470 310 470 "" 0 0 0 "">
  <770 180 770 230 "" 0 0 0 "">
  <770 290 770 300 "" 0 0 0 "">
</Wires>
<Diagrams>
  <Rect 610 770 240 160 3 #c0c0c0 1 00 1 -1 0.5 1 1 -1 1 1 1 -1 1 1 315 0 225 "" "" "">
	<"dBS11" #0000ff 0 3 0 0 0>
  </Rect>
</Diagrams>
<Paintings>
</Paintings>
