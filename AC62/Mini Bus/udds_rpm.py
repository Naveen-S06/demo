import pandas as pd
rpm_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 160.73618529166666, 316.1144977402778, 460.77706450277776, 616.1553769513888, 766.1758165569444, 905.4805104763889, 926.912001848611, 969.7749845930555, 1109.0796785125, 1162.6584069430553, 1200.1635168444438, 1205.5213896874998, 1184.0898983152777, 1151.9426612569443, 1119.7954241986108, 1093.0060599833334, 1060.858822925, 910.8383833194443, 798.3230536152777, 798.3230536152777, 814.3966721444443, 830.470290673611, 857.2596548888887, 916.1962561624999, 1023.3537130236109, 1130.511169884722, 1216.237135373611, 1226.9528810597224, 1216.237135373611, 1210.8792625305555, 1141.2269155708332, 1017.9958401805553, 916.1962561624999, 846.5439092027776, 846.5439092027776, 948.3434932208331, 1060.858822925, 1157.3005340999998, 1243.026499588889, 1296.6052280194442, 1318.0367193916666, 1334.1103379208328, 1339.4682107638887, 1318.0367193916666, 1312.6788465486109, 1323.394592234722, 1328.7524650777775, 1323.394592234722, 1318.0367193916666, 1318.0367193916666, 1344.8260836069442, 1371.6154478222224, 1376.9733206652777, 1360.899702136111, 1334.1103379208328, 1339.4682107638887, 1360.899702136111, 1393.0469391944443, 1393.0469391944443, 1376.9733206652777, 1398.4048120375, 1430.5520490958334, 1473.4150318402774, 1532.3516331138887, 1569.8567430152775, 1596.6461072305553, 1612.7197257597222, 1628.7933442888886, 1644.8669628180553, 1644.8669628180553, 1634.1512171319441, 1628.7933442888886, 1623.435471445833, 1628.7933442888886, 1650.2248356611108, 1628.7933442888886, 1602.003980073611, 1580.5724887013887, 1596.6461072305553, 1623.435471445833, 1644.8669628180553, 1655.5827085041665, 1660.940581347222, 1655.5827085041665, 1628.7933442888886, 1596.6461072305553, 1602.003980073611, 1618.0775986027777, 1644.8669628180553, 1671.6563270333331, 1703.8035640916662, 1725.2350554638886, 1735.9508011499995, 1725.2350554638886, 1698.4456912486107, 1532.3516331138887, 1355.5418292930553, 1178.732025472222, 1001.9222216513888, 825.1124178305554, 648.3026140097221, 471.4928101888888, 294.6830063680555, 117.8732025472222, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 176.80980382083334, 353.6196076416667, 530.4294114625, 707.2392152833334, 884.0490191041666, 1060.858822925, 1189.4477711583331, 1301.9631008625, 1382.3311935083332, 1414.4784305666667, 1376.9733206652777, 1344.8260836069442, 1323.394592234722, 1339.4682107638887, 1350.1839564499999, 1360.899702136111, 1382.3311935083332, 1457.3414133111107, 1419.836303409722, 1285.8894823333333, 1216.237135373611, 1039.4273315527776, 948.3434932208331, 921.5541290055555, 969.7749845930555, 996.5643488083334, 1071.5745686111109, 1189.4477711583331, 1312.6788465486109, 1462.6992861541662, 1634.1512171319441, 1794.887402423611, 1939.549969186111, 1998.4865704597216, 2105.644027320833, 2169.9385014375, 2255.6644669263887, 2330.6746867291663, 2416.4006522180553, 2464.621507805555, 2507.4844905499995, 2544.989600451389, 2544.989600451389, 2534.273854765277, 2528.9159819222223, 2518.2002362361113, 2518.2002362361113, 2518.2002362361113, 2518.2002362361113, 2518.2002362361113, 2528.9159819222223, 2539.6317276083328, 2566.4210918236104, 2598.568328881944, 2630.715565940278, 2652.1470573125, 2678.9364215277774, 2711.0836585861107, 2732.515149958333, 2759.3045141736106, 2796.809624075, 2850.3883525055558, 2898.6092080930553, 2925.3985723083324, 2941.4721908374995, 2946.8300636805548, 2941.4721908374995, 2925.3985723083324, 2925.3985723083324, 2936.1143179944434, 2952.187936523611, 2973.619427895833, 2984.335173581944, 3005.766664954166, 3016.4824106402775, 3032.556029169444, 3037.9139020124994, 3037.9139020124994, 3027.1981563263885, 3027.1981563263885, 3027.1981563263885, 3027.1981563263885, 3027.1981563263885, 3027.1981563263885, 3021.8402834833323, 3005.766664954166, 2989.693046424999, 2952.187936523611, 2925.3985723083324, 2903.9670809361114, 2893.251335249999, 2877.177716720833, 2871.819843877778, 2887.893462406944, 2893.251335249999, 2898.6092080930553, 2898.6092080930553, 2882.535589563889, 2861.1040981916667, 2839.672606819444, 2818.241115447222, 2791.451751231944, 2807.5253697611106, 2786.0938783888887, 2780.7360055458325, 2770.0202598597216, 2759.3045141736106, 2764.6623870166663, 2775.3781327027773, 2791.451751231944, 2812.8832426041668, 2839.672606819444, 2866.461971034722, 2893.251335249999, 2941.4721908374995, 2968.2615550527776, 2978.9773007388885, 3000.408792111111, 3000.408792111111, 2989.693046424999, 2957.5458093666666, 2920.040699465277, 2871.819843877778, 2812.8832426041668, 2759.3045141736106, 2759.3045141736106, 2759.3045141736106, 2737.8730228013883, 2684.294294370833, 2678.9364215277774, 2684.294294370833, 2678.9364215277774, 2657.5049301555555, 2652.1470573125, 2652.1470573125, 2652.1470573125, 2630.715565940278, 2603.926201725, 2577.136837509722, 2528.9159819222223, 2469.9793806486105, 2411.0427793749996, 2346.748305258333, 2282.4538311416663, 2223.517229868055, 2159.2227557513884, 2062.7810445763885, 1982.4129519305552, 1885.9712407555553, 1810.9610209527777, 1741.3086739930552, 1687.7299455624998, 1639.5090899749998, 1634.1512171319441, 1607.3618529166665, 1553.7831244861109, 1473.4150318402774, 1328.7524650777775, 1151.9426612569443, 1076.9324414541668, 1023.3537130236109, 991.2064759652776, 910.8383833194443, 830.470290673611, 669.7341053819443, 578.6502670499999, 428.62982744444435, 251.82002362361106, 75.01021980277777, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 53.57872843055554, 230.3885322513888, 407.19833607222216, 584.0081398930555, 760.8179437138888, 926.912001848611, 1071.5745686111109, 1205.5213896874998, 1269.8158638041664, 1350.1839564499999, 1425.1941762527779, 1505.562268898611, 1607.3618529166665, 1650.2248356611108, 1693.0878184055553, 1719.8771826208329, 1757.3822925222216, 1800.2452752666666, 1848.4661308541665, 1853.824003697222, 1869.8976222263886, 1864.5397493833327, 1848.4661308541665, 1859.1818765402777, 1902.044859284722, 1928.8342234999996, 1928.8342234999996, 1928.8342234999996, 1928.8342234999996, 1928.8342234999996, 1928.8342234999996, 1934.1920963430553, 1950.2657148722217, 1955.6235877152776, 1950.2657148722217, 1928.8342234999996, 1880.6133679124996, 1827.0346394819444, 1794.887402423611, 1682.372072719444, 1553.7831244861109, 1376.9733206652777, 1232.3107539027776, 1087.6481871402775, 937.6277475347222, 776.8915622430554, 642.9447411666666, 466.1349373458333, 289.32513352499996, 112.51532970416666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 139.30469391944442, 316.1144977402778, 492.92430156111106, 669.7341053819443, 846.5439092027776, 1023.3537130236109, 1200.1635168444438, 1339.4682107638887, 1371.6154478222224, 1473.4150318402774, 1553.7831244861109, 1607.3618529166665, 1612.7197257597222, 1607.3618529166665, 1591.2882343874999, 1569.8567430152775, 1543.0673788, 1500.2043960555554, 1339.4682107638887, 1162.6584069430553, 985.8486031222221, 809.0387993013888, 632.2289954805556, 455.4191916597222, 278.60938783888884, 101.79958401805554, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 176.80980382083334, 353.6196076416667, 530.4294114625, 707.2392152833334, 884.0490191041666, 1060.858822925, 1237.668626745833, 1414.4784305666667, 1489.4886503694443, 1559.1409973291666, 1687.7299455624998, 1768.0980382083333, 1800.2452752666666, 1864.5397493833327, 1880.6133679124996, 1907.4027321277777, 1934.1920963430553, 1928.8342234999996, 1934.1920963430553, 1939.549969186111, 1928.8342234999996, 1912.7606049708338, 1928.8342234999996, 1928.8342234999996, 1907.4027321277777, 1902.044859284722, 1896.6869864416662, 1885.9712407555553, 1885.9712407555553, 1885.9712407555553, 1885.9712407555553, 1885.9712407555553, 1885.9712407555553, 1875.2554950694444, 1880.6133679124996, 1885.9712407555553, 1902.044859284722, 1885.9712407555553, 1875.2554950694444, 1875.2554950694444, 1875.2554950694444, 1864.5397493833327, 1853.824003697222, 1848.4661308541665, 1794.887402423611, 1714.5193097777774, 1612.7197257597222, 1500.2043960555554, 1366.2575749791665, 1205.5213896874998, 1060.858822925, 884.0490191041666, 707.2392152833334, 551.8609028347221, 385.7668447, 214.31491372222217, 53.57872843055554, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.29447411666665, 187.52554950694443, 294.6830063680555, 348.2617347986111, 455.4191916597222, 514.3557929333332, 562.5766485208333, 637.5868683236112, 750.1021980277777, 857.2596548888887, 948.3434932208331, 1017.9958401805553, 1076.9324414541668, 1125.1532970416665, 1178.732025472222, 1232.3107539027776, 1275.1737366472223, 1312.6788465486109, 1334.1103379208328, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1371.6154478222224, 1382.3311935083332, 1393.0469391944443, 1371.6154478222224, 1350.1839564499999, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1307.3209737055554, 1237.668626745833, 1060.858822925, 884.0490191041666, 707.2392152833334, 530.4294114625, 353.6196076416667, 176.80980382083334, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 176.80980382083334, 353.6196076416667, 530.4294114625, 696.5234695972222, 782.249435086111, 857.2596548888887, 910.8383833194443, 910.8383833194443, 910.8383833194443, 937.6277475347222, 948.3434932208331, 948.3434932208331, 937.6277475347222, 910.8383833194443, 905.4805104763889, 889.4068919472222, 910.8383833194443, 916.1962561624999, 910.8383833194443, 889.4068919472222, 884.0490191041666, 884.0490191041666, 889.4068919472222, 910.8383833194443, 942.9856203777776, 991.2064759652776, 1028.7115858666664, 1082.290314297222, 1125.1532970416665, 1130.511169884722, 1135.8690427277777, 1157.3005340999998, 1178.732025472222, 1200.1635168444438, 1205.5213896874998, 1205.5213896874998, 1205.5213896874998, 1216.237135373611, 1269.8158638041664, 1344.8260836069442, 1393.0469391944443, 1419.836303409722, 1446.6256676249996, 1398.4048120375, 1221.5950082166664, 1044.7852043958333, 867.9754005749998, 691.1655967541666, 514.3557929333332, 337.54598911249997, 160.73618529166666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 107.15745686111109, 241.10427793749994, 417.9140817583333, 546.5030299916667, 669.7341053819443, 750.1021980277777, 819.7545449874999, 937.6277475347222, 1050.1430772388887, 1125.1532970416665, 1189.4477711583331, 1248.3843724319443, 1312.6788465486109, 1355.5418292930553, 1371.6154478222224, 1393.0469391944443, 1398.4048120375, 1403.7626848805553, 1403.7626848805553, 1414.4784305666667, 1419.836303409722, 1419.836303409722, 1393.0469391944443, 1366.2575749791665, 1264.4579909611111, 1146.5847884138886, 991.2064759652776, 878.6911462611109, 776.8915622430554, 621.5132497944445, 466.1349373458333, 310.75662489722214, 187.52554950694443, 107.15745686111109, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 75.01021980277777, 176.80980382083334, 235.74640509444444, 348.2617347986111, 492.92430156111106, 605.4396312652777, 723.3128338124998, 782.249435086111, 878.6911462611109, 894.7647647902777, 884.0490191041666, 884.0490191041666, 975.132857436111, 1028.7115858666664, 1076.9324414541668, 1151.9426612569443, 1205.5213896874998, 1205.5213896874998, 1184.0898983152777, 1216.237135373611, 1248.3843724319443, 1259.1001181180557, 1205.5213896874998, 1157.3005340999998, 1098.3639328263887, 964.4171117499998, 803.6809264583333, 642.9447411666666, 482.2085558749999, 332.18811626944444, 241.10427793749994, 160.73618529166666, 112.51532970416666, 26.78936421527777, 26.78936421527777, 171.45193097777778, 348.2617347986111, 514.3557929333332, 669.7341053819443, 750.1021980277777, 857.2596548888887, 964.4171117499998, 1050.1430772388887, 1151.9426612569443, 1237.668626745833, 1312.6788465486109, 1366.2575749791665, 1419.836303409722, 1451.9835404680557, 1478.7729046833329, 1494.8465232124995, 1516.278014584722, 1532.3516331138887, 1532.3516331138887, 1516.278014584722, 1510.9201417416662, 1500.2043960555554, 1473.4150318402774, 1435.909921938889, 1366.2575749791665, 1259.1001181180557, 1151.9426612569443, 1017.9958401805553, 884.0490191041666, 798.3230536152777, 669.7341053819443, 503.6400472472221, 332.18811626944444, 160.73618529166666, 80.36809264583333, 80.36809264583333, 26.78936421527777, 0.0, 160.73618529166666, 337.54598911249997, 514.3557929333332, 691.1655967541666, 846.5439092027776, 937.6277475347222, 985.8486031222221, 1044.7852043958333, 1109.0796785125, 1178.732025472222, 1243.026499588889, 1339.4682107638887, 1419.836303409722, 1473.4150318402774, 1500.2043960555554, 1516.278014584722, 1548.4252516430554, 1548.4252516430554, 1548.4252516430554, 1543.0673788, 1526.9937602708333, 1516.278014584722, 1516.278014584722, 1516.278014584722, 1510.9201417416662, 1478.7729046833329, 1473.4150318402774, 1473.4150318402774, 1473.4150318402774, 1473.4150318402774, 1473.4150318402774, 1473.4150318402774, 1478.7729046833329, 1500.2043960555554, 1526.9937602708333, 1607.3618529166665, 1660.940581347222, 1714.5193097777774, 1768.0980382083333, 1768.0980382083333, 1800.2452752666666, 1821.6767666388887, 1837.7503851680553, 1832.3925123249999, 1821.6767666388887, 1821.6767666388887, 1816.3188937958325, 1800.2452752666666, 1773.4559110513885, 1768.0980382083333, 1741.3086739930552, 1714.5193097777774, 1709.1614369347224, 1693.0878184055553, 1687.7299455624998, 1639.5090899749998, 1607.3618529166665, 1602.003980073611, 1602.003980073611, 1602.003980073611, 1602.003980073611, 1585.9303615444442, 1580.5724887013887, 1580.5724887013887, 1569.8567430152775, 1548.4252516430554, 1510.9201417416662, 1484.1307775263888, 1446.6256676249996, 1366.2575749791665, 1269.8158638041664, 1178.732025472222, 1098.3639328263887, 1028.7115858666664, 1028.7115858666664, 1076.9324414541668, 1119.7954241986108, 1146.5847884138886, 1178.732025472222, 1210.8792625305555, 1243.026499588889, 1285.8894823333333, 1339.4682107638887, 1393.0469391944443, 1425.1941762527779, 1425.1941762527779, 1435.909921938889, 1446.6256676249996, 1457.3414133111107, 1489.4886503694443, 1505.562268898611, 1543.0673788, 1548.4252516430554, 1553.7831244861109, 1559.1409973291666, 1553.7831244861109, 1505.562268898611, 1473.4150318402774, 1446.6256676249996, 1382.3311935083332, 1339.4682107638887, 1312.6788465486109, 1328.7524650777775, 1344.8260836069442, 1366.2575749791665, 1376.9733206652777, 1403.7626848805553, 1441.2677947819445, 1473.4150318402774, 1489.4886503694443, 1521.6358874277776, 1553.7831244861109, 1564.498870172222, 1559.1409973291666, 1553.7831244861109, 1548.4252516430554, 1526.9937602708333, 1505.562268898611, 1500.2043960555554, 1500.2043960555554, 1478.7729046833329, 1457.3414133111107, 1425.1941762527779, 1446.6256676249996, 1473.4150318402774, 1489.4886503694443, 1500.2043960555554, 1489.4886503694443, 1500.2043960555554, 1500.2043960555554, 1500.2043960555554, 1484.1307775263888, 1468.0571589972221, 1441.2677947819445, 1425.1941762527779, 1419.836303409722, 1419.836303409722, 1419.836303409722, 1409.120557723611, 1403.7626848805553, 1403.7626848805553, 1387.6890663513886, 1371.6154478222224, 1371.6154478222224, 1387.6890663513886, 1382.3311935083332, 1366.2575749791665, 1318.0367193916666, 1259.1001181180557, 1189.4477711583331, 1157.3005340999998, 1157.3005340999998, 1162.6584069430553, 1210.8792625305555, 1253.7422452749997, 1285.8894823333333, 1296.6052280194442, 1307.3209737055554, 1334.1103379208328, 1344.8260836069442, 1350.1839564499999, 1355.5418292930553, 1366.2575749791665, 1350.1839564499999, 1339.4682107638887, 1339.4682107638887, 1339.4682107638887, 1323.394592234722, 1312.6788465486109, 1301.9631008625, 1301.9631008625, 1312.6788465486109, 1339.4682107638887, 1339.4682107638887, 1318.0367193916666, 1318.0367193916666, 1291.2473551763887, 1312.6788465486109, 1344.8260836069442, 1371.6154478222224, 1344.8260836069442, 1285.8894823333333, 1178.732025472222, 1076.9324414541668, 905.4805104763889, 728.6707066555554, 551.8609028347221, 375.05109901388886, 198.24129519305552, 21.431491372222222, 0.0, 0.0, 0.0, 107.15745686111109, 283.96726068194437, 460.77706450277776, 637.5868683236112, 814.3966721444443, 937.6277475347222, 996.5643488083334, 1071.5745686111109, 1130.511169884722, 1178.732025472222, 1232.3107539027776, 1312.6788465486109, 1409.120557723611, 1473.4150318402774, 1505.562268898611, 1521.6358874277776, 1526.9937602708333, 1526.9937602708333, 1526.9937602708333, 1484.1307775263888, 1473.4150318402774, 1457.3414133111107, 1435.909921938889, 1419.836303409722, 1393.0469391944443, 1376.9733206652777, 1350.1839564499999, 1285.8894823333333, 1178.732025472222, 1151.9426612569443, 1151.9426612569443, 1168.016279786111, 1205.5213896874998, 1232.3107539027776, 1221.5950082166664, 1221.5950082166664, 1232.3107539027776, 1216.237135373611, 1216.237135373611, 1216.237135373611, 1259.1001181180557, 1285.8894823333333, 1318.0367193916666, 1328.7524650777775, 1344.8260836069442, 1366.2575749791665, 1371.6154478222224, 1366.2575749791665, 1339.4682107638887, 1291.2473551763887, 1269.8158638041664, 1243.026499588889, 1226.9528810597224, 1205.5213896874998, 1178.732025472222, 1157.3005340999998, 1098.3639328263887, 937.6277475347222, 760.8179437138888, 584.0081398930555, 407.19833607222216, 230.3885322513888, 53.57872843055554, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.29447411666665, 214.31491372222217, 391.1247175430555, 567.9345213638888, 744.7443251847221, 910.8383833194443, 991.2064759652776, 1071.5745686111109, 1168.016279786111, 1232.3107539027776, 1285.8894823333333, 1328.7524650777775, 1371.6154478222224, 1419.836303409722, 1435.909921938889, 1468.0571589972221, 1494.8465232124995, 1516.278014584722, 1500.2043960555554, 1473.4150318402774, 1446.6256676249996, 1446.6256676249996, 1409.120557723611, 1312.6788465486109, 1205.5213896874998, 1151.9426612569443, 1103.7218056694442, 964.4171117499998, 803.6809264583333, 659.0183596958333, 594.7238855791666, 567.9345213638888, 535.7872843055554, 508.99792009027766, 487.5664287180555, 466.1349373458333, 460.77706450277776, 471.4928101888888, 482.2085558749999, 466.1349373458333, 460.77706450277776, 428.62982744444435, 375.05109901388886, 267.8936421527777, 225.03065940833332, 139.30469391944442, 53.57872843055554, 0.0, 5.357872843055555, 32.147237058333324, 85.72596548888887, 192.88342235, 369.6932261708332, 535.7872843055554, 685.807723911111, 750.1021980277777, 776.8915622430554, 857.2596548888887, 969.7749845930555, 1071.5745686111109, 1125.1532970416665, 1135.8690427277777, 1141.2269155708332, 1146.5847884138886, 1162.6584069430553, 1205.5213896874998, 1232.3107539027776, 1275.1737366472223, 1312.6788465486109, 1339.4682107638887, 1334.1103379208328, 1328.7524650777775, 1339.4682107638887, 1360.899702136111, 1382.3311935083332, 1393.0469391944443, 1414.4784305666667, 1425.1941762527779, 1441.2677947819445, 1446.6256676249996, 1446.6256676249996, 1446.6256676249996, 1441.2677947819445, 1435.909921938889, 1435.909921938889, 1419.836303409722, 1414.4784305666667, 1393.0469391944443, 1366.2575749791665, 1318.0367193916666, 1259.1001181180557, 1151.9426612569443, 1071.5745686111109, 937.6277475347222, 857.2596548888887, 750.1021980277777, 573.2923942069443, 396.48259038611104, 219.67278656527773, 42.86298274444444, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 112.51532970416666, 289.32513352499996, 466.1349373458333, 642.9447411666666, 819.7545449874999, 996.5643488083334, 1130.511169884722, 1232.3107539027776, 1259.1001181180557, 1232.3107539027776, 1205.5213896874998, 1071.5745686111109, 894.7647647902777, 717.9549609694445, 541.145157148611, 364.3353533277777, 187.52554950694443, 10.71574568611111, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.71574568611111, 80.36809264583333, 187.52554950694443, 348.2617347986111, 525.0715386194444, 642.9447411666666, 691.1655967541666, 696.5234695972222, 675.0919782249999, 685.807723911111, 701.8813424402776, 701.8813424402776, 750.1021980277777, 830.470290673611, 910.8383833194443, 996.5643488083334, 1055.5009500819442, 1125.1532970416665, 1151.9426612569443, 1168.016279786111, 1168.016279786111, 1151.9426612569443, 1135.8690427277777, 1151.9426612569443, 1168.016279786111, 1178.732025472222, 1173.3741526291665, 1162.6584069430553, 1151.9426612569443, 1151.9426612569443, 1146.5847884138886, 1076.9324414541668, 1044.7852043958333, 1028.7115858666664, 1050.1430772388887, 1060.858822925, 1071.5745686111109, 1044.7852043958333, 937.6277475347222, 830.470290673611, 696.5234695972222, 535.7872843055554, 428.62982744444435, 321.4723705833333, 214.31491372222217, 133.94682107638886, 37.505109901388884, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 53.57872843055554, 53.57872843055554, 53.57872843055554, 53.57872843055554, 53.57872843055554, 85.72596548888887, 160.73618529166666, 214.31491372222217, 267.8936421527777, 337.54598911249997, 428.62982744444435, 535.7872843055554, 562.5766485208333, 508.99792009027766, 455.4191916597222, 407.19833607222216, 471.4928101888888, 589.366012736111, 750.1021980277777, 910.8383833194443, 1044.7852043958333, 1125.1532970416665, 1168.016279786111, 1189.4477711583331, 1232.3107539027776, 1264.4579909611111, 1291.2473551763887, 1312.6788465486109, 1312.6788465486109, 1285.8894823333333, 1259.1001181180557, 1259.1001181180557, 1259.1001181180557, 1259.1001181180557, 1259.1001181180557, 1259.1001181180557, 1285.8894823333333, 1291.2473551763887, 1312.6788465486109, 1323.394592234722, 1339.4682107638887, 1360.899702136111, 1371.6154478222224, 1376.9733206652777, 1393.0469391944443, 1403.7626848805553, 1446.6256676249996, 1489.4886503694443, 1516.278014584722, 1553.7831244861109, 1559.1409973291666, 1553.7831244861109, 1500.2043960555554, 1323.394592234722, 1146.5847884138886, 969.7749845930555, 792.9651807722221, 616.1553769513888, 439.34557313055547, 262.5357693097222, 85.72596548888887, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 80.36809264583333, 257.1778964666666, 433.9877002874999, 610.7975041083332, 707.2392152833334, 809.0387993013888, 900.1226376333333, 980.4907302791664, 1044.7852043958333, 1087.6481871402775, 1141.2269155708332, 1173.3741526291665, 1184.0898983152777, 1200.1635168444438, 1178.732025472222, 1157.3005340999998, 1130.511169884722, 1098.3639328263887, 1071.5745686111109, 1050.1430772388887, 991.2064759652776, 937.6277475347222, 884.0490191041666, 830.470290673611, 750.1021980277777, 589.366012736111, 428.62982744444435, 278.60938783888884, 133.94682107638886, 0.0, 0.0, 0.0]

df = pd.DataFrame({'RPM': rpm_values})

# Specify the file path to save the Excel file
file_path = 'udds_values.xlsx'

# Save the DataFrame to an Excel file
df.to_excel(file_path, index=False)

print(f"RPM values saved to '{file_path}' successfully.")