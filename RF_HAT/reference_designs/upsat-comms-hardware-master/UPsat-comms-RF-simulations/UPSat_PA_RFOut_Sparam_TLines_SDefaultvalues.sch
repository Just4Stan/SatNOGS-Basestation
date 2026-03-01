<Qucs Schematic 0.0.18>
<Properties>
  <View=-170,-867,3625,1083,0.549226,467,108>
  <Grid=10,10,1>
  <DataSet=UPSat_PA_RFOut_Sparam_TLines_SDefaultvalues.dat>
  <DataDisplay=UPSat_PA_RFOut_Sparam_TLines_SDefaultvalues.dpl>
  <OpenDisplay=1>
  <Script=UPSat_PA_RFOut_Sparam_TLines_SDefaultvalues.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
  <.ID -20 -16 SUB>
  <Line -20 20 40 0 #000080 2 1>
  <Line 20 20 0 -40 #000080 2 1>
  <Line -20 -20 40 0 #000080 2 1>
  <Line -20 20 0 -40 #000080 2 1>
</Symbol>
<Components>
  <CLIN CLTC5 1 1020 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "1.1 mm" 1 "Metal" 1 "yes" 0>
  <CLIN CLTC6 1 640 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "1.429 mm" 1 "Metal" 1 "yes" 0>
  <GND * 1 850 -30 0 0 0 0>
  <CLIN CLTC7 1 370 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "3.955 mm" 1 "Metal" 1 "yes" 0>
  <GND * 1 240 130 0 0 0 0>
  <MVIA MS16 5 300 90 20 0 0 0 "SubstTC1" 0 "0.6 mm" 0 "26.85" 0>
  <GND * 1 240 240 0 0 0 0>
  <MVIA MS17 5 300 200 20 0 0 0 "SubstTC1" 0 "0.6 mm" 1 "26.85" 0>
  <MVIA MS18 1 300 320 20 0 0 0 "SubstTC1" 0 "0.6 mm" 1 "26.85" 0>
  <GND * 1 240 360 0 0 0 0>
  <GND * 1 250 10 0 0 0 1>
  <GND * 1 1270 -240 0 0 0 1>
  <CGAP CL8 1 1360 -240 28 -26 0 1 "SubstTC1" 1 "0.7 mm" 1 "0.25 mm" 1 "0.62 mm" 1>
  <MVIA MS6 1 1240 -420 20 0 0 0 "SubstTC1" 1 "0.6 mm" 1 "26.85" 0>
  <CLIN CLTC4 1 1460 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "1.2 mm" 1 "Metal" 1 "yes" 0>
  <MTEE MS2 1 1700 -70 -26 66 0 2 "SubstTC1" 1 "0.56 mm" 0 "0.56 mm" 0 "0.56mm" 0 "Wheeler" 0 "Yamashita" 0 "26.85" 0 "showNumbers" 0>
  <CGAP CL11 1 1700 -240 28 -26 0 1 "SubstTC1" 1 "0.7 mm" 1 "0.25 mm" 1 "0.62 mm" 1>
  <MVIA MS19 1 1720 -420 20 0 0 0 "SubstTC1" 1 "0.6 mm" 1 "26.85" 0>
  <CLIN CLTC3 1 1910 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "1.195mm" 1 "Metal" 1 "yes" 0>
  <MTEE MS3 1 2140 -70 -26 49 0 2 "SubstTC1" 1 "0.56 mm" 1 "0.56 mm" 0 "0.56mm" 0 "Wheeler" 0 "Yamashita" 0 "26.85" 0 "showNumbers" 0>
  <MVIA MS20 1 2160 -420 20 0 0 0 "SubstTC1" 1 "0.6 mm" 1 "26.85" 0>
  <CGAP CL12 1 2280 -230 28 -26 0 1 "SubstTC1" 1 "0.7 mm" 1 "0.25 mm" 1 "0.62 mm" 1>
  <GND * 1 2570 -30 0 0 0 0>
  <CGAP CL5 1 2570 80 -26 28 0 0 "SubstTC1" 1 "0.65 mm" 0 "0.25 mm" 0 "0.62 mm" 0>
  <CLIN CLTC1 1 2810 -70 -26 25 0 0 "SubstTC1" 1 "0.56 mm" 1 "0.25 mm" 1 "10.795 mm" 1 "Metal" 1 "yes" 0>
  <Pac P1 1 3050 150 18 -26 0 1 "1" 1 "50 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
  <GND * 1 3050 370 0 0 0 0>
  <CLIN CLTC2 1 2360 -70 -26 25 0 0 "SubstTC1" 1 "0.56mm" 1 "0.25 mm" 1 "1.330 mm" 1 "Metal" 1 "yes" 0>
  <CGAP CL1 1 850 70 -26 28 0 0 "SubstTC1" 1 "0.7 mm" 0 "0.25 mm" 0 "0.62 mm" 0>
  <Eqn Eqn1 1 190 600 -28 15 0 0 "dBS11=dB(S[1,1])" 1 "yes" 0>
  <SUBST SubstTC1 1 450 640 -30 24 0 0 "4.5" 1 "0.36 mm" 1 "36 um" 1 "0.0145" 1 "1.67785e-08" 1 "0" 1>
  <CGAP C157_DNM 1 490 -250 28 -26 0 1 "SubstTC1" 1 "0.7 mm" 1 "0.25 mm" 1 "0.62 mm" 1>
  <MVIA MS11 5 510 -410 20 0 0 0 "SubstTC1" 0 "0.6 mm" 1 "26.85" 0>
  <MVIA MS12 5 510 -340 20 0 0 0 "SubstTC1" 0 "0.6 mm" 1 "26.85" 0>
  <GND * 1 2190 -230 0 0 0 1>
  <.SP SP1 1 0 590 0 95 0 0 "log" 1 "100MHz" 1 "2GHz" 1 "1000" 1 "no" 0 "1" 0 "2" 0 "no" 0 "no" 0>
  <SPfile C151 1 240 90 -26 -37 0 0 "/home/manos/MurataSparameters0603ForPA/GRM1882C1H470JA01.s2p" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile C152 1 240 200 -26 -37 0 0 "/home/manos/MurataSparameters0603ForPA/GRM1882C1H102JA01.s2p" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile C153 1 240 320 -26 -37 0 0 "/home/manos/MurataSparameters0603ForPA/GRM188R61A335KE15.s2p" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <MTEE MS1 1 1220 -70 -26 49 0 2 "SubstTC1" 1 "0.56 mm" 1 "0.56 mm" 0 "0.56mm" 0 "Wheeler" 0 "Yamashita" 0 "26.85" 0 "showNumbers" 0>
  <MTEE MS21 5 210 -70 -26 -100 0 0 "SubstTC1" 0 "0.56 mm" 0 "0.56 mm" 0 "0.56mm" 0 "Wheeler" 0 "Yamashita" 0 "26.85" 0 "showNumbers" 0>
  <GND * 1 20 360 0 0 0 0>
  <R R1 1 20 170 15 -26 0 1 "2.6 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <L L34 1 20 50 10 -26 0 1 "0.00055 uH" 1 "" 0>
  <SPfile L31 1 210 10 -35 -26 0 1 "" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile L33 1 850 -70 -26 -37 0 0 "" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile C160 5 1220 -240 -36 -26 0 1 "" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile C164 5 2140 -240 -42 -26 0 1 "" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
  <SPfile C169 1 2570 -70 -26 -37 0 0 "" 0 "rectangular" 0 "linear" 0 "open" 0 "2" 0>
</Components>
<Wires>
  <670 -70 790 -70 "" 0 0 0 "">
  <910 -70 990 -70 "" 0 0 0 "">
  <400 -70 490 -70 "" 0 0 0 "">
  <1360 -280 1360 -270 "" 0 0 0 "">
  <1220 -280 1360 -280 "" 0 0 0 "">
  <1220 -200 1360 -200 "" 0 0 0 "">
  <1360 -210 1360 -200 "" 0 0 0 "">
  <1220 -420 1220 -280 "" 0 0 0 "">
  <1490 -70 1670 -70 "" 0 0 0 "">
  <1700 -210 1700 -100 "" 0 0 0 "">
  <1700 -420 1700 -270 "" 0 0 0 "">
  <1730 -70 1880 -70 "" 0 0 0 "">
  <1940 -70 2110 -70 "" 0 0 0 "">
  <2140 -190 2140 -100 "" 0 0 0 "">
  <2140 -420 2140 -270 "" 0 0 0 "">
  <2140 -270 2280 -270 "" 0 0 0 "">
  <2280 -270 2280 -260 "" 0 0 0 "">
  <2140 -190 2280 -190 "" 0 0 0 "">
  <2280 -200 2280 -190 "" 0 0 0 "">
  <2610 -70 2610 80 "" 0 0 0 "">
  <2600 80 2610 80 "" 0 0 0 "">
  <2530 -70 2530 80 "" 0 0 0 "">
  <2530 80 2540 80 "" 0 0 0 "">
  <2610 -70 2780 -70 "" 0 0 0 "">
  <3050 180 3050 370 "" 0 0 0 "">
  <2840 -70 3050 -70 "" 0 0 0 "">
  <3050 -70 3050 120 "" 0 0 0 "">
  <2390 -70 2530 -70 "" 0 0 0 "">
  <2170 -70 2330 -70 "" 0 0 0 "">
  <2570 -40 2570 -30 "" 0 0 0 "">
  <2600 -70 2610 -70 "" 0 0 0 "">
  <2530 -70 2540 -70 "" 0 0 0 "">
  <1250 -240 1270 -240 "" 0 0 0 "">
  <1220 -280 1220 -270 "" 0 0 0 "">
  <1220 -210 1220 -200 "" 0 0 0 "">
  <910 -70 910 70 "" 0 0 0 "">
  <880 70 910 70 "" 0 0 0 "">
  <790 -70 790 70 "" 0 0 0 "">
  <790 70 820 70 "" 0 0 0 "">
  <490 -70 610 -70 "" 0 0 0 "">
  <490 -220 490 -70 "" 0 0 0 "">
  <490 -340 490 -280 "" 0 0 0 "">
  <490 -410 490 -340 "" 0 0 0 "">
  <850 -40 850 -30 "" 0 0 0 "">
  <880 -70 910 -70 "" 0 0 0 "">
  <790 -70 820 -70 "" 0 0 0 "">
  <240 10 250 10 "" 0 0 0 "">
  <210 40 210 90 "" 0 0 0 "">
  <210 90 210 200 "" 0 0 0 "">
  <240 120 240 130 "" 0 0 0 "">
  <270 90 280 90 "" 0 0 0 "">
  <240 230 240 240 "" 0 0 0 "">
  <270 200 280 200 "" 0 0 0 "">
  <240 350 240 360 "" 0 0 0 "">
  <270 320 280 320 "" 0 0 0 "">
  <210 200 210 320 "" 0 0 0 "">
  <1220 -200 1220 -100 "" 0 0 0 "">
  <1050 -70 1190 -70 "" 0 0 0 "">
  <1250 -70 1430 -70 "" 0 0 0 "">
  <240 -70 340 -70 "" 0 0 0 "">
  <210 -40 210 -20 "" 0 0 0 "">
  <20 -70 180 -70 "" 0 0 0 "">
  <20 -70 20 20 "" 0 0 0 "">
  <20 80 20 140 "" 0 0 0 "">
  <20 200 20 360 "" 0 0 0 "">
  <2170 -240 2170 -230 "" 0 0 0 "">
  <2170 -230 2190 -230 "" 0 0 0 "">
  <2140 -210 2140 -190 "" 0 0 0 "">
</Wires>
<Diagrams>
  <Rect 780 730 240 160 3 #c0c0c0 1 00 1 0 5e+08 2e+09 1 -14.1947 10 1.17568 1 -1 1 1 315 0 225 "" "" "">
	<"dBS11" #0000ff 0 3 0 0 0>
	  <Mkr 4.3076e+08 92 -64 3 0 0>
	  <Mkr 4.35959e+08 92 -128 3 0 0>
  </Rect>
</Diagrams>
<Paintings>
  <Text 140 70 12 #000000 0 "47pF">
  <Text 140 300 12 #000000 0 "3.3uF">
  <Text 80 0 12 #000000 0 "Ferrite Bead">
  <Text 1590 -220 12 #000000 0 "C159 DNM">
  <Text 2040 -220 12 #000000 0 "C164 2pF">
  <Text 1110 -230 12 #000000 0 "C160 22pF">
  <Text 2610 -110 12 #000000 0 "56pF">
  <Text 880 -110 12 #000000 0 "2.7nH">
  <Text 1290 550 12 #000000 0 "Type\nCapacitor\nCapacitor\nCapacitor\nInductor\nCapacitor\nFerrite Bead\nCapacitor\nCapacitor\nCapacitor\n\n\n">
  <Text 1440 550 12 #000000 0 "Kicad/Qucs annotation\nC169\nC159\nC160\nL33\nC157\nL31\nC151\nC152\nC153\n">
  <Text 140 190 12 #000000 0 "1000pF">
  <Text 1770 550 12 #000000 0 "Component\nGRM1882C1H560JA01\n-\nGRM1882C1H220JA01\nLQG18HN2N7S00\n-\nNR3012T1R0\nGRM1882C1H470JA01\nGRM1882C1H102JA01\nGRM188R61A335KE15">
  <Text 1640 550 12 #000000 0 "Value\n56pF\nDo not mount\n22pF\n2.7nH\nDo not mount\n1uH\n47pF\n1000pF\n3.3uF\n">
</Paintings>
