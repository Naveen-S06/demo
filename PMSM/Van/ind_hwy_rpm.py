import pandas as pd
rpm_values = [0.0, 0.5593908698540452, 45.43974340688244, 127.28530793710338, 238.74033478653593, 372.44907428119785, 521.0557767471074, 677.2046925102861, 833.540071896746, 982.7061652325089, 1117.3472228435921, 1232.0999780603513, 1329.5710962304638, 1414.3597257059348, 1491.0650148387867, 1564.286111981029, 1637.8032556935455, 1712.121045372694, 1786.9251706237044, 1861.9013210518026, 1936.7351862622309, 2010.6078564638417, 2080.682024280081, 2143.6157829380027, 2196.0672256646817, 2234.6944456871843, 2257.8010065714593, 2270.272353238917, 2278.6394009498645, 2289.433064964605, 2309.184260543426, 2341.922132100607, 2381.668740666307, 2419.9443764246653, 2448.2693295598156, 2458.1638902559025, 2444.1466950726476, 2412.729766072122, 2373.4234716919723, 2335.738180369856, 2309.184260543426, 2299.739651182523, 2299.252573385702, 2296.0388187837134, 2278.41417900729, 2234.6944456871843, 2155.983922685931, 2044.5409627932065, 1905.4124310305, 1743.6451924192802, 1564.286111981029, 1374.2951315794874, 1188.284500447431, 1022.7795446599127, 894.3055902919623, 819.3879634186321, 808.8785186649347, 850.935224855735, 928.0425793658884, 1022.6850795702246, 1117.3472228435921, 1198.0126784720599, 1264.6618033865961, 1320.7741264293904, 1369.8291764426442, 1415.3064822685521, 1460.4072565589104, 1507.2194472039446, 1557.5526859034817, 1613.2166043573543, 1676.0208342653882, 1746.878896474891, 1823.119868423069, 1901.1767166946227, 1977.4824078742276, 2048.4699085465895, 2111.4553523026207, 2167.287540758212, 2217.6984425354603, 2264.420026256499, 2309.184260543426, 2352.2783938836387, 2388.2107942256116, 2410.0451093831066, 2410.844987169875, 2383.6740753996673, 2323.8101020840845, 2235.3871160261056, 2124.7532462265485, 1998.2566216862372, 1862.2453714059893, 1723.4029644750242, 1589.7542303361543, 1469.659338520587, 1371.478458559531, 1303.5717599841926, 1269.5521347366393, 1254.0433644024104, 1236.9219529779011, 1198.0644044594922, 1117.3472228435921, 984.0639453525639, 825.1762421126472, 677.0628164760707, 576.1023717950483, 558.6736114217969, 648.4956130346893, 818.6489516166833, 1029.5545764769174, 1241.6334369244998, 1415.3064822685521, 1521.2283683074281, 1570.988576796411, 1586.4102959800118, 1589.3167141027538, 1601.531019409153, 1638.650185058076, 1691.3663238658896, 1744.1453335632975, 1781.4531118810098, 1787.7555565497473, 1752.9529821113538, 1688.6833703521745, 1612.0191198697003, 1540.0326292614157, 1489.796297124794, 1473.001684718886, 1479.8170039490376, 1495.0296293821368, 1503.4269355850852, 1489.796297124794, 1443.989528512018, 1376.1162040330191, 1301.3503379179137, 1234.8659443968115, 1191.837037699834, 1183.883161478198, 1208.4059770675265, 1259.2526752245471, 1330.2704467059834, 1415.3064822685521, 1508.4226372579606, 1604.5394253758202, 1698.7920249127233, 1786.315614159247, 1862.2453714059893, 1922.9224908694014, 1969.5122304693896, 2004.3858640517335, 2029.914665462207, 2048.4699085465895, 2061.853527749587, 2069.5900999116825, 2070.634862472274, 2063.943052870775, 2048.4699085465895, 2024.2420086175068, 1995.5712989148424, 1967.8410669482867, 1946.4346002275174, 1936.7351862622309, 1942.4958000656306, 1960.9481666649806, 1987.6936985910693, 2018.3338083746798, 2048.4699085465895, 2074.6839310537634, 2097.4798855079075, 2118.3423009368976, 2138.755706368615, 2160.204630830949, 2183.3716344806485, 2205.731401989979, 2223.9566491600854, 2234.7200917921077, 2234.6944456871843, 2221.289986417652, 2194.8672286405667, 2156.52424678419, 2107.3591152767794, 2048.4699085465895, 1982.0839826619247, 1914.9458202513256, 1854.9291855833765, 1809.907842926651, 1787.7555565497473, 1792.8791599463902, 1815.8177635108912, 1843.6435468626846, 1863.4286896212354, 1862.2453714059893, 1831.5613540318611, 1780.426728095658, 1722.2871663896385, 1670.58834170609, 1638.7759268372706, 1637.2046891671557, 1663.865774446506, 1713.6594230177834, 1781.4858752234593, 1862.2453714059893, 1950.4633320816777, 2039.1658984621604, 2121.0043919329228, 2188.6301338794374, 2234.6944456871843, 2253.161281600926, 2243.245127302559, 2205.4731013332616, 2140.3723222342082, 2048.4699085465895, 1933.1642079456528, 1809.338484642979, 1694.7472319842173, 1607.1449433150156, 1564.286111981029, 1577.6827856396576, 1633.8772291953082, 1713.1692618641396, 1795.858702862316, 1862.2453714059893, 1896.85008476474, 1901.0776524218188, 1880.5538819139056, 1840.9045807776554, 1787.7555565497473, 1727.1155628401543, 1666.5251375520265, 1613.9077646618266, 1577.1869281460108, 1564.286111981029, 1580.3556103909875, 1619.4529585905418, 1672.8625020419918, 1731.8685862076254, 1787.7555565497473, 1832.982875165202, 1864.710470688998, 1881.273388390691, 1881.0066735398325, 1862.2453714059893, 1823.780924548298, 1766.2303646842474, 1690.6671208209277, 1598.164621965417, 1489.796297124794, 1368.9981361828911, 1248.6563725305778, 1144.0198004354722, 1070.3372141651923, 1042.8574079873567, 1071.0939650049154, 1141.6196236621731, 1235.2719112387722, 1332.8883550143548, 1415.3064822685521, 1468.4482225728073, 1498.5731146657486, 1517.0250995778174, 1535.1481183394303, 1564.286111981029, 1611.907156869809, 1669.9758707200394, 1726.5810065827527, 1769.8113175089725, 1787.7555565497473, 1771.7705108910375, 1726.2851042585517, 1658.9962945129232, 1577.6010395147912, 1489.796297124794, 1403.0215311098107, 1323.6862288617183, 1257.9423836786175, 1211.9419888586183, 1191.837037699834, 1200.647587962179, 1228.8659552528227, 1263.8525196407213, 1292.9676611948598, 1303.5717599841926, 1287.467573060312, 1254.2273654052763, 1217.8657789837328, 1192.3974557603656, 1191.837037699834, 1226.2881277138238, 1290.2101725021282, 1374.1515797115549, 1468.6607569889197, 1564.286111981029, 1653.2431352623867, 1734.4156491182466, 1808.354558761539, 1875.6107694052198, 1936.7351862622309, 1991.3091774442723, 2035.0359626581574, 2062.6492245094537, 2068.8826456037436, 2048.4699085465895, 1998.6537206802186, 1926.7128882934462, 1842.4352424117383, 1755.6086140605669, 1676.0208342653882, 1610.9482759136638, 1557.621479340754, 1510.7595267440274, 1465.081500320841, 1415.3064822685521, 1359.2209737085639, 1306.8811514584863, 1271.4106112599638, 1265.932948854647, 1303.5717599841926, 1391.5658932045933, 1513.615208329238, 1647.5348179858654, 1771.1398348022017, 1862.2453714059893, 1905.1340250505496, 1909.958331491558, 1893.3383111102933, 1871.893984288013, 1862.2453714059893, 1876.8191422783113, 1911.2685644503483, 1957.0535549002843, 2005.6340306063, 2048.4699085465895, 2079.282123337986, 2100.835680151986, 2118.1566017987157, 2136.270911088323, 2160.204630830949, 2192.516737924675, 2225.898025619362, 2250.5722412528094, 2256.763132162815, 2234.6944456871843, 2178.6215840601917, 2098.9265691019655, 2010.0230775291207, 1926.3247860582605, 1862.2453714059893, 1828.9675106949762, 1824.74988267213, 1844.6201664904136, 1883.606041302788, 1936.7351862622309, 1998.988846218238, 2065.162528806501, 2130.0053073592585, 2188.266255208742, 2234.6944456871843, 2265.6679333780326, 2284.080697869577, 2294.455700001323, 2301.3159006127707, 2309.184260543426, 2321.7754758604788, 2339.571183541842, 2362.244755793117, 2389.469564819901, 2420.918982827785, 2455.4867857850354, 2488.948364710546, 2516.29951438586, 2532.536029592543, 2532.6537051121445, 2513.787229891346, 2481.6268695373597, 2444.00178382252, 2408.7411325191756, 2383.6740753996673, 2374.515017742215, 2378.519346848626, 2390.827695526568, 2406.580696583729, 2420.918982827785, 2429.9400442080864, 2433.5687992406015, 2432.6870235829597, 2428.176492892816, 2420.918982827785, 2412.1693097889606, 2404.6744531512227, 2401.5544330328985, 2405.9292695523077, 2420.918982827785, 2448.3864098999006, 2485.165655498218, 2526.8336412745325, 2568.9672888806545, 2607.143519968386, 2637.7214773845726, 2660.1891887562347, 2674.816902905424, 2681.8748686542026, 2681.6333348246208, 2674.5765633523106, 2662.0448686271097, 2645.592578152451, 2626.7740194317344, 2607.143519968386, 2588.1069299136598, 2570.4761900101944, 2554.914763648468, 2542.0861142189565, 2532.6537051121445, 2527.0649774947233, 2524.9032836382503, 2525.535953590481, 2528.3303173991917, 2532.6537051121445, 2537.6941759867063, 2541.9227061186366, 2543.6310008132796, 2541.110765375998, 2532.6537051121445, 2517.186671453042, 2496.1771003338877, 2471.7275738158514, 2445.9406739600918, 2420.918982827785, 2397.5956058043234, 2372.2257415720023, 2339.8951121373543, 2295.689439506902, 2234.6944456871843, 2154.8467762241526, 2065.4867708214415, 1978.8056927221314, 1906.9948051692916, 1862.2453714059893, 1853.3675001601478, 1875.6466820991006, 1920.987253375036, 1981.2935501401366, 2048.4699085465895, 2115.261703706725, 2177.778466573438, 2232.9707670597636, 2277.7891750787508, 2309.184260543426, 2325.3154290791113, 2329.1774291602483, 2324.9738449735355, 2316.9082607056866, 2309.184260543426, 2305.0963950149994, 2304.3030800148244, 2305.5536977788665, 2307.5976305430772, 2309.184260543426, 2309.4045541214537, 2308.715814041016, 2307.916927171573, 2307.8067803825556, 2309.184260543426, 2311.795033240903, 2311.171878930868, 2301.794356786465, 2278.142025980851, 2234.6944456871843, 2168.610639066202, 2087.767485228958, 2002.721327274083, 1924.0285083002175, 1862.2453714059893, 1825.4146472486354, 1811.5246167197536, 1816.0499482695518, 1834.465310348232, 1862.2453714059893, 1895.386722704292, 1931.973646749694, 1970.6123488599976, 2009.9090343530283, 2048.4699085465895, 2085.3270979548274, 2121.2164138772173, 2157.299588809538, 2194.738355247599, 2234.6944456871843, 2277.593781324633, 2320.919038158414, 2361.417080887527, 2395.834774210982, 2420.918982827785, 2434.7420584007714, 2440.678300448087, 2443.4274954517246, 2447.689429893673, 2458.1638902559025, 2478.2620783182306, 2506.24085705174, 2539.0685047253314, 2573.7132996079113, 2607.143519968386, 2636.5807848074487, 2660.2600760529604, 2676.6697163645663, 2684.2980284019004, 2681.6333348246208, 2668.333057845665, 2648.7310178911503, 2628.3301349404896, 2612.6333289730933, 2607.143519968386, 2615.7114408144766, 2635.5790760343093, 2662.3362230595317, 2691.572679321794, 2718.878242252745, 2740.81296920453, 2757.8179572113518, 2771.3045632278986, 2782.6841442088758, 2793.3680571089803, 2804.3265618487217, 2814.7655302118505, 2823.4497369479163, 2829.143956806479, 2830.6129645371043, 2827.1177038461697, 2819.903794267369, 2810.7130242912212, 2801.287182408256, 2793.3680571089803, 2788.345695128513, 2786.2031761802677, 2786.5718382222567, 2789.0830192124963, 2793.3680571089803, 2798.777250896884, 2803.5367436699325, 2805.5916395490417, 2802.8870426550898, 2793.3680571089803, 2775.263847640741, 2747.9398214168964, 2711.0454462131106, 2664.2301898050564, 2607.143519968386, 2540.665415831079, 2470.5979019303836, 2403.9735141558403, 2347.824788397003, 2309.184260543426, 2292.559993801102, 2292.362160641761, 2300.4764608535847, 2308.78859422475, 2309.184260543426, 2295.5634760129587, 2269.88352249733, 2236.115998275686, 2198.2325016271725, 2160.204630830949, 2125.536498599285, 2095.8622753770414, 2072.3486460421846, 2056.162295472706, 2048.4699085465895, 2049.6978774752424, 2057.311423803808, 2068.035476410863, 2078.594964174972, 2085.7148159747076, 2086.8669835869514, 2082.5115103818334, 2073.855462627778, 2062.1059065932145, 2048.4699085465895, 2034.588369164474, 2023.8375267560707, 2020.0274540387281, 2026.9682237297827, 2048.4699085465895, 2086.158179934994, 2132.9211042548754, 2179.4623465946174, 2216.4855720425944, 2234.6944456871843, 2227.4351773655258, 2198.624155909755, 2154.8203149007504, 2102.5825879194003, 2048.4699085465895, 1998.1044141253199, 1953.3610570470507, 1915.1779934653573, 1884.4933795338093, 1862.2453714059893, 1848.9869691411345, 1843.7305484211404, 1845.1033288335705, 1851.7325299660004, 1862.2453714059893, 1874.7669007620757, 1885.4134777267027, 1889.7992900132858, 1883.5385253352422, 1862.2453714059893, 1823.9278599493766, 1776.1693987309882, 1728.947239526847, 1692.2386341129766, 1676.0208342653882, 1687.157072885897, 1720.0545073794663, 1766.0062762768337, 1816.3055181087623, 1862.2453714059893, 1896.2424284029305, 1915.2070961486677, 1917.1732353959314, 1900.1747068974582, 1862.2453714059893, 1803.614804846916, 1733.2954438362322, 1662.4954401625937, 1602.4229456146425, 1564.286111981029, 1555.9418803634899, 1571.8423491160406, 1603.0884059058083, 1640.7809383998865, 1676.0208342653882, 1701.2854531834464, 1714.5580428912542, 1715.1983231400334, 1702.5660136810102, 1676.0208342653882, 1635.7474198695015, 1585.2300663700596, 1528.7779848688735, 1470.7003864677656, 1415.3064822685521, 1366.6987393500028, 1328.1526486987245, 1302.7369572782873, 1293.5204120522515, 1303.5717599841926, 1334.4720126076102, 1381.8512397357883, 1439.8517757519403, 1502.6159550392829, 1564.286111981029, 1619.2030815151843, 1662.5017007988265, 1689.5153075438327, 1695.5772394620633, 1676.0208342653882, 1626.873162876634, 1546.936229062463, 1435.7057698004812, 1292.6775220683223, 1117.3472228435921, 912.7922794126565, 696.4167802968292, 489.20648432617327, 312.1471503307429, 186.22453714059893, 125.44888914435451, 115.92839296484348, 136.7957207834547, 167.1835447815769, 186.22453714059893, 178.60032415517648, 151.18834857303662, 116.41500725517145, 86.70669706257429, 74.48981485623968, 89.48474438138331, 130.58781692011604, 193.98935063877357, 275.8796637036891, 372.44907428119785, 478.30993592069854, 581.7627437038475, 669.5300280953655, 728.3343195599778, 744.8981485623966, 712.5782430754696, 651.268121104506, 587.4954981629514, 547.788089764236, 558.6736114217969, 638.70463102075, 774.5331259329428, 944.8359259018973, 1128.289860671142, 1303.5717599841926, 1453.171779911141, 1574.833381828298, 1670.1133534385376, 1740.568482444727, 1787.7555565497473, 1814.296065042455, 1827.0703035556417, 1834.0232693081068, 1843.0999595186252, 1862.2453714059893, 1895.8017584098568, 1933.7003988533827, 1962.269827280603, 1967.8385782355379, 1936.7351862622309, 1858.2058853553447, 1733.1677073121364, 1565.4553833805126, 1358.9036448083623, 1117.3472228435921, 849.9609613847672, 587.2801549331358, 365.1802106406181, 219.53653565913245, 186.22453714059893, 287.86596478233463, 494.0679384632512, 761.1839206076573, 1045.5673736398676, 1303.5717599841926, 1501.0862740262767, 1642.1430379971441, 1740.3099060891432, 1809.15473249464, 1862.2453714059893, 1910.9053103410756, 1957.480570119932, 2002.0728048881174, 2044.7836687911902, 2085.7148159747076, 2124.707594580183, 2160.5621287329136, 2191.818236554159, 2217.0157361651654, 2234.6944456871843, 2243.998140597321, 2246.4864257960603, 2244.322863539728, 2239.671016084657, 2234.6944456871843, 2231.1889467408555, 2229.479242188073, 2229.522287108458, 2231.27503658162, 2234.6944456871843, 2239.4127465623674, 2243.763279574765, 2245.7546621495703, 2243.3955117119785, 2234.6944456871843, 2217.8369448356666, 2191.71594325899, 2155.4012383940167, 2107.962627677602, 2048.4699085465895, 1975.6101480391337, 1886.5394915985853, 1778.031354269582, 1646.859151096768, 1489.796297124794, 1306.4600619549435, 1107.8431334151455, 907.7820538899692, 720.1133657639963, 558.6736114217969, 433.75562690564914, 341.4774228886524, 274.4133037016098, 225.13757367532403, 186.22453714059893, 152.64069145801332, 128.92130610725616, 121.99384359779144, 138.78576643908394, 186.22453714059893, 268.19214502923745, 376.3886867016492, 499.46878557192224, 626.087065054143, 744.8981485623966, 846.8973777256192, 932.4429670321626, 1004.2338491852045, 1064.9689568879555, 1117.3472228435921, 1163.7093652669798, 1204.9632444195886, 1241.6585060745645, 1274.3447960050567, 1303.5717599841926, 1329.875061301914, 1353.7344333153123, 1375.6156268982925, 1395.9843929247397, 1415.3064822685521, 1433.865871705942, 1451.2194416224245, 1466.7422983058343, 1479.8095480440086, 1489.796297124794, 1496.8187307099229, 1503.9573494568035, 1515.0337328967448, 1533.8694605610463, 1564.286111981029, 1608.216603068298, 1660.0391952556824, 1712.2434863562958, 1757.31907418328, 1787.7555565497473, 1797.7907994634897, 1788.6557417108793, 1763.329590272968, 1724.791552130789, 1676.0208342653882, 1620.5534604279658, 1564.152721450367, 1513.1387249346112, 1473.8315784827098, 1452.5513896966702, 1453.4304864597577, 1471.8500777802321, 1501.0035929475962, 1534.0844612513633, 1564.286111981029, 1585.2493357859041, 1592.4043687544565, 1581.6288083349398, 1548.8002519756308, 1489.796297124794, 1404.1704273639318, 1306.1796708075951, 1213.756941703566, 1144.8351542996377, 1117.3472228435921, 1143.3118927325927, 1211.091233961242, 1303.1331476735054, 1401.8855350133674, 1489.796297124794, 1553.502283671515, 1596.3961383963597, 1626.0594535618955, 1650.0738214307116, 1676.0208342653882, 1709.5792368828218, 1748.8163843171396, 1789.8967841568085, 1828.9849439902714, 1862.2453714059893, 1886.2854226611396, 1899.483848687854, 1900.6622490869934, 1888.6422234594183, 1862.2453714059893, 1821.0206638171658, 1767.426556741802, 1704.6488775183548, 1635.8734534852815, 1564.286111981029, 1491.8060613329092, 1415.2860338235885, 1330.3121427245808, 1232.4705013074126, 1117.3472228435921, 982.483851284273, 833.2436532990941, 676.9453262373396, 520.9075674482835, 372.44907428119785, 238.8885440853599, 127.54467421004583, 45.73616200453056, 0.7817048180901505, 0.0, 0.0]

df = pd.DataFrame({'RPM': rpm_values})

# Specify the file path to save the Excel file
file_path = 'indhwy_values.xlsx'

# Save the DataFrame to an Excel file
df.to_excel(file_path, index=False)

print(f"RPM values saved to '{file_path}' successfully.")