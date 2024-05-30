import pandas as pd
rpm_values = [0.0, 0.6912700564079547, 56.15238946648223, 157.29345389663416, 295.0245590132762, 460.25580048281927, 643.8972739716747, 836.8590751462581, 1030.0512996729733, 1214.3840432182358, 1380.7674014484562, 1522.5736908366878, 1643.0241110825175, 1747.8020826921634, 1842.5910261718625, 1933.0743620278388, 2023.9235389090466, 2115.762118035345, 2208.2016887693217, 2300.8538404735596, 2393.3301625106624, 2484.618683023337, 2571.213185274886, 2648.983891308714, 2713.8010231682497, 2761.5348028969124, 2790.088850713306, 2805.500379536696, 2815.840000461549, 2829.1783245823285, 2853.585962993479, 2894.0419509932713, 2943.159020695187, 2990.45832841652, 3025.461030474558, 3037.6882831866037, 3020.3664643525954, 2981.5428377030853, 2932.969888451262, 2886.4001018103277, 2853.585962993479, 2841.9147398872356, 2841.3128310713623, 2837.341417798962, 2815.5616813231113, 2761.5348028969124, 2664.267881666731, 2526.551688352004, 2354.6229115654683, 2154.7182399198373, 1933.0743620278388, 1698.2920607481558, 1468.4284959232853, 1263.9049216417015, 1105.1425919918481, 1012.5627610621985, 999.5756623103005, 1051.5474466700146, 1146.8332444443138, 1263.7881859361396, 1380.7674014484562, 1480.4501404195328, 1562.8121288288776, 1632.1532117913052, 1692.7732344216438, 1748.972041834714, 1804.705548525434, 1862.553946509148, 1924.7534971812972, 1993.5404619373287, 2071.1511021726847, 2158.7143058290144, 2252.929467032781, 2349.388606456674, 2443.6837447733515, 2531.406902655508, 2609.2414788070137, 2678.2363840566222, 2740.531907264262, 2798.2683372899046, 2853.585962993479, 2906.8397531254373, 2951.2433959981736, 2978.2252598145947, 2979.213712777588, 2945.637123090045, 2871.6599195982094, 2762.390773721719, 2625.6744175235485, 2469.355583066682, 2301.279002414097, 2129.7038058149337, 1964.5467162629827, 1816.13885493819, 1694.8113430205026, 1610.8953016898677, 1568.8553801766877, 1549.6903399132352, 1528.5324703822355, 1480.514061066395, 1380.7674014484562, 1216.0619267711527, 1019.7156553172671, 836.6837511296172, 711.9213782510034, 690.383700724229, 801.3816870478531, 1011.6495235434086, 1272.2772009882133, 1534.3547101595493, 1748.972041834714, 1879.8655406078262, 1941.3569663402945, 1960.4144327103425, 1964.0060533962092, 1979.099942076126, 2024.9701360938477, 2090.114367455355, 2155.3362918321377, 2201.4395648956925, 2209.22784231753, 2166.2203874381757, 2086.798894274233, 1992.0606645113462, 1903.1029998351542, 1841.0232019312798, 1820.2691759168567, 1828.6912406350284, 1847.490318360404, 1857.8673313676086, 1841.0232019312798, 1784.4172592367174, 1700.5424601120285, 1608.1501682959977, 1525.991747527399, 1472.8185615450227, 1462.9895193480581, 1493.2937109773209, 1556.1277717340379, 1643.888336919431, 1748.972041834714, 1864.0407946172088, 1982.81759474861, 2099.2907145467057, 2207.4484263292693, 2301.279002414097, 2376.261055312259, 2433.834558307971, 2476.9298248787445, 2508.4771685020864, 2531.406902655508, 2547.9457768130583, 2557.506284435021, 2558.797354978202, 2550.5279178994265, 2531.406902655508, 2501.4671545237707, 2466.0371820635696, 2431.769409654756, 2405.316261677167, 2393.3301625106624, 2400.448869739223, 2423.2514737633483, 2456.3023981876813, 2494.1660666168586, 2531.406902655508, 2563.801011665442, 2591.971226037193, 2617.7520599184595, 2642.9780274569425, 2669.483642800354, 2698.1123830654738, 2725.7435772454514, 2748.2655173025278, 2761.5664951989365, 2761.5348028969124, 2744.970175523238, 2712.318120862809, 2664.935589865078, 2604.1795334794956, 2531.406902655508, 2449.3701637595723, 2366.403844826246, 2292.237989307093, 2236.6026406536607, 2209.22784231753, 2215.5593607599626, 2243.905854481052, 2278.291704990568, 2302.7412937983213, 2301.279002414097, 2263.3610749610248, 2200.1712060156456, 2128.3249527678227, 2064.437872407455, 2025.125522124405, 2023.1838573397542, 2056.1304263993507, 2117.663175880238, 2201.4800523594745, 2301.279002414097, 2410.294787152216, 2519.90942580616, 2621.0417521393315, 2704.6105999151173, 2761.5348028969124, 2784.3552874484385, 2772.101350334715, 2725.424380921089, 2644.9757685729, 2531.406902655508, 2388.9173082519287, 2235.899053315932, 2094.2923415189575, 1986.037376532444, 1933.0743620278388, 1949.6293682940213, 2019.071932089644, 2117.0574567907997, 2219.2413457735893, 2301.279002414097, 2344.041949477795, 2349.2661872876465, 2323.903835556002, 2274.907013995179, 2209.22784231753, 2134.291667754749, 2059.4167476159305, 1994.3945667295368, 1949.0166099240232, 1933.0743620278388, 1952.9323248065266, 2001.2470677739248, 2067.248177380951, 2140.1652400785115, 2209.22784231753, 2265.117726788907, 2304.325261143456, 2324.7929692719717, 2324.463375065249, 2301.279002414097, 2253.746370435397, 2182.6279791504053, 2089.2503238064887, 1974.9398996509963, 1841.0232019312798, 1691.746272277291, 1543.0333378493965, 1413.7281721905588, 1322.6745488437384, 1288.7162413518977, 1323.6097074669005, 1410.7621417762612, 1526.4934230764031, 1647.1234301637508, 1748.972041834714, 1814.6422123673935, 1851.8691979665768, 1874.6713303187414, 1897.0669411103358, 1933.0743620278388, 1991.9223056760104, 2063.6810083328123, 2133.6310871944984, 2187.053159457313, 2209.22784231753, 2189.474242447258, 2133.2654244220703, 2050.112942293392, 1949.5283501126514, 1841.0232019312798, 1733.7908521905422, 1635.7518568910837, 1554.508572423363, 1497.663355177851, 1472.8185615450227, 1483.7062429589605, 1518.5772310282389, 1561.8120524050225, 1597.7912337415114, 1610.8953016898677, 1590.994472407152, 1549.9177200699103, 1504.983708359527, 1473.5111009574316, 1472.8185615450227, 1515.3916677945049, 1594.3836533413053, 1698.1146658116438, 1814.904852831749, 1933.0743620278388, 2043.0034470656985, 2143.312785769343, 2234.6831620023295, 2317.7953596282455, 2393.3301625106624, 2460.770244206419, 2514.805837045515, 2548.9290630512123, 2556.6320442467913, 2531.406902655508, 2469.846300128333, 2380.9450578270153, 2276.798536741001, 2169.5020978597463, 2071.1511021726847, 1990.737363753709, 1924.8385090143934, 1866.9286174507745, 1810.4817685588741, 1748.972041834714, 1679.6640950031046, 1614.9848987040411, 1571.152001806294, 1564.3829531786414, 1610.8953016898677, 1719.6344905341662, 1870.4575402074272, 2035.9493655309661, 2188.6948813260824, 2301.279002414097, 2354.27886998768, 2360.240530724967, 2339.7022576754694, 2313.202323888677, 2301.279002414097, 2319.2886124308193, 2361.8596576362925, 2418.438687857545, 2478.472252921603, 2531.406902655508, 2569.4832507061296, 2596.1181659997833, 2617.522581282601, 2639.9074293007407, 2669.483642800354, 2709.4134902416745, 2750.664582941306, 2781.155867929932, 2788.806292238237, 2761.5348028969124, 2692.242484575807, 2593.7589725013604, 2483.896039539191, 2380.465458554902, 2301.279002414097, 2260.155720125202, 2254.9437652679103, 2279.4985675647204, 2327.675556738132, 2393.3301625106624, 2470.2604331827943, 2552.0348913670086, 2632.164678253785, 2704.1609350335957, 2761.5348028969124, 2799.8104447370984, 2822.564110259057, 2835.3850708705804, 2843.8625979794565, 2853.585962993479, 2869.145620098241, 2891.136754590523, 2919.155734544913, 2952.798928035989, 2991.662703138324, 3034.380037989343, 3075.7303509777757, 3109.529670555177, 3129.594025173121, 3129.7394432831697, 3106.425102465446, 3066.68277681433, 3020.1873895527583, 2976.6138639036817, 2945.637123090045, 2934.3187719250304, 2939.267141582882, 2954.477244828059, 2973.9440944250505, 2991.662703138324, 3002.810524715833, 3007.2947768393465, 3006.205118174099, 3000.6312073853524, 2991.662703138324, 2980.850251056871, 2971.588444599267, 2967.7328641823924, 2973.139090223115, 2991.662703138324, 3025.6057130885524, 3071.0558492089804, 3122.547270378429, 3174.614135475738, 3221.790603379736, 3259.5774667109927, 3287.3420530570284, 3305.4183237470875, 3314.1402401104283, 3313.841763476295, 3305.121322946053, 3289.635218709427, 3269.304218728278, 3246.0490909644304, 3221.790603379736, 3198.2660423078346, 3176.478767569578, 3157.248657357616, 3141.3955898645936, 3129.7394432831697, 3122.833145265915, 3120.161821305103, 3120.94364635291, 3124.396795361537, 3129.7394432831697, 3135.9682302969622, 3141.1936574899114, 3143.3046911759543, 3140.190297669058, 3129.7394432831697, 3110.6259793240934, 3084.6632970650244, 3054.4496727710125, 3022.583382707094, 2991.662703138324, 2962.840723696938, 2931.4897874839203, 2891.5370509674563, 2836.9096706157234, 2761.5348028969124, 2662.8626472570545, 2552.435575053562, 2445.3190006217224, 2356.5783382968098, 2301.279002414097, 2290.3081287591567, 2317.8397389187603, 2373.869575929977, 2448.39338282987, 2531.406902655508, 2613.94519653217, 2691.2005979379733, 2759.4047584392374, 2814.7893296023017, 2853.585962993479, 2873.5201349375056, 2878.292620792756, 2873.0980206759864, 2863.1309347039655, 2853.585962993479, 2848.53436278563, 2847.5540198188887, 2849.099476956078, 2851.625277059999, 2853.585962993479, 2853.8581918808563, 2853.007077892566, 2852.019849460598, 2851.8837350169047, 2853.585962993479, 2856.8122383710597, 2856.0421723255386, 2844.4538525815647, 2815.2253668638014, 2761.5348028969124, 2679.8714093873655, 2579.969078968833, 2474.8728652567906, 2377.627821866722, 2301.279002414097, 2255.76525140768, 2238.600576929325, 2244.1927779541825, 2266.9496534573946, 2301.279002414097, 2342.2335925156317, 2387.4460663181753, 2435.194035094085, 2483.755110115744, 2531.406902655508, 2576.953358227653, 2621.3037593140825, 2665.893722638567, 2712.1588649249175, 2761.5348028969124, 2814.547870796466, 2868.087272937926, 2918.1329311537456, 2960.6647672763916, 2991.662703138324, 3008.74463769611, 3016.080378402729, 3019.477709835286, 3024.7444165708857, 3037.6882831866037, 3062.5247192892666, 3097.0996346045363, 3137.6665638877844, 3180.4790418943912, 3221.790603379736, 3258.1678501715983, 3287.4296523873836, 3307.7079472168957, 3317.134671849927, 3313.841763476295, 3297.4058798876267, 3273.1825612828675, 3247.9720684627846, 3228.5746622281454, 3221.790603379736, 3232.3784542828475, 3256.9299835609163, 3289.9952614019144, 3326.1243579938123, 3359.8673435245823, 3386.973291716424, 3407.9872904285585, 3424.6534310544193, 3438.7158049874647, 3451.9185036211406, 3465.460530489696, 3478.3605376905707, 3489.0920894619912, 3496.128750042194, 3497.944083669428, 3493.624797844092, 3484.710173115302, 3473.352633294351, 3461.7046021925385, 3451.9185036211406, 3445.712094763511, 3443.0644662911423, 3443.5200422475896, 3446.623246676412, 3451.9185036211406, 3458.602941813457, 3464.4845087374165, 3467.0238565652326, 3463.681637469075, 3451.9185036211406, 3429.546136507611, 3395.7803348705947, 3350.187926766201, 3292.3357402505503, 3221.790603379736, 3139.6399547485207, 3053.053675106279, 2970.722255741013, 2901.3361879407407, 2853.585962993479, 2833.042442482131, 2832.797969169128, 2842.8252561118024, 2853.0970163674797, 2853.585962993479, 2836.7540106000592, 2805.019880009165, 2763.291493595666, 2716.476773734433, 2669.483642800354, 2626.642325548262, 2589.972256252968, 2560.9151715692237, 2540.9128081518124, 2531.406902655508, 2532.9243713814426, 2542.332849216244, 2555.585150692911, 2568.634090344428, 2577.432482703787, 2578.856279575043, 2573.4739818464964, 2562.777227677489, 2548.2576552273695, 2531.406902655508, 2514.2527211541046, 2500.967314046854, 2496.2589976903087, 2504.8360884410063, 2531.406902655508, 2577.9803719282154, 2635.7678888049923, 2693.2814610695586, 2739.033096505628, 2761.5348028969124, 2752.56412587549, 2716.9607624668265, 2662.829947544731, 2598.27691598302, 2531.406902655508, 2469.167491814409, 2413.875665225522, 2366.690754033045, 2328.772089381168, 2301.279002414097, 2284.8948657121573, 2278.3992176001984, 2280.0956378392057, 2288.2877061901786, 2301.279002414097, 2316.752544745158, 2329.90910531042, 2335.3288947101505, 2327.5921235446194, 2301.279002414097, 2253.9279465898635, 2194.9101900272535, 2136.555171352621, 2091.192329192318, 2071.1511021726847, 2084.9127645705357, 2125.5659332645114, 2182.3510607836943, 2244.5085996571956, 2301.279002414097, 2343.291035099063, 2366.726717819045, 2369.1563841965394, 2348.150367854053, 2301.279002414097, 2228.8259874710825, 2141.9284865070244, 2054.437028975855, 1980.2021443314848, 1933.0743620278388, 1922.7629362041703, 1942.4120197409761, 1981.0244902041156, 2027.6032251594038, 2071.1511021726847, 2102.3719809638746, 2118.7736498691784, 2119.5648793788873, 2103.9544399833, 2071.1511021726847, 2021.3830295395908, 1958.9559580855082, 1889.1950169141708, 1817.4253351293255, 1748.972041834714, 1688.9047811768198, 1641.2712574731388, 1609.8636900839233, 1598.4742983694134, 1610.8953016898677, 1649.0804429303446, 1707.6295590752084, 1779.30401063363, 1856.8651581147833, 1933.0743620278388, 2000.9382809322256, 2054.4447655882936, 2087.8269648066594, 2095.3180273979196, 2071.1511021726847, 2010.4166222157833, 1911.6341577089556, 1774.1805631081447, 1597.43269286933, 1380.7674014484562, 1127.9876102428484, 860.6005084152412, 604.5393520697369, 385.7373973104266, 230.12790024140963, 155.0240902175606, 143.25908959686856, 169.0459939881011, 206.59789900002556, 230.12790024140963, 220.70624103221596, 186.83175553719272, 143.86042563228094, 107.14823319342335, 92.05116009656398, 110.58121901830485, 161.3745458378927, 239.72330723523658, 340.9196698902427, 460.25580048281927, 591.073888050151, 718.9162110585295, 827.3750703315237, 900.0427666927075, 920.5116009656399, 880.5721166210396, 804.8078277181752, 726.0004909634763, 676.9318630633544, 690.383700724229, 789.2824322086904, 957.1331720040282, 1167.585706153698, 1394.289820701162, 1610.8953016898677, 1795.7642721836394, 1946.1082033277046, 2063.8509032876514, 2150.9161802290605, 2209.22784231753, 2242.0254080118657, 2257.8112369437595, 2266.403399038152, 2277.6199642199535, 2301.279002414097, 2342.7464749580495, 2389.5799088394724, 2424.8847224585797, 2431.7663342155683, 2393.3301625106624, 2296.287187387125, 2141.77063546051, 1934.5192949894508, 1679.2719542325558, 1380.7674014484562, 1050.3434957283268, 725.7343794935831, 451.27326599819963, 271.29336849615015, 230.12790024140963, 355.7318012089392, 610.5469182576502, 940.6368249674398, 1292.0650949182125, 1610.8953016898677, 1854.9748471763464, 2029.2864465278096, 2150.5966432084497, 2235.671980682479, 2301.279002414097, 2361.4107645596628, 2418.966374044218, 2474.0714504849598, 2526.8516134990846, 2577.432482703787, 2625.618003273944, 2669.925422615104, 2708.5503136905027, 2739.6882494633633, 2761.5348028969124, 2773.0318902680565, 2776.1068011083794, 2773.4331682631255, 2767.6846245775496, 2761.5348028969124, 2757.202865096428, 2755.0900891711585, 2755.143282146136, 2757.309251046375, 2761.5348028969124, 2767.3654667277115, 2772.741659588474, 2775.2025205338355, 2772.2871886184357, 2761.5348028969124, 2740.7030621722806, 2708.423904240988, 2663.5478266478767, 2604.9253269377773, 2531.406902655508, 2441.370090347505, 2331.300582566628, 2197.2111108673253, 2035.1144068040549, 1841.0232019312798, 1614.4645352505893, 1369.022675552192, 1121.796199073431, 889.8836820516622, 690.383700724229, 536.0156785479354, 421.9824278574159, 339.10760820676137, 278.2148791500621, 230.12790024140963, 188.62649549832847, 159.3151467920865, 150.75450045738472, 171.50520282892498, 230.12790024140963, 331.41978036020436, 465.12419617333427, 617.2210419994908, 773.6902121573632, 920.5116009656399, 1046.5576569472573, 1152.2710454421667, 1240.9869859945366, 1316.0406981485776, 1380.7674014484562, 1438.0596500984402, 1489.03933694301, 1534.3856894867306, 1574.777935234169, 1610.8953016898677, 1643.3997374350574, 1672.8840753575976, 1699.9238694220353, 1725.094673592894, 1748.972041834714, 1771.9068998645366, 1793.351660409477, 1812.5341079491611, 1828.6820269632153, 1841.0232019312798, 1849.7012092462369, 1858.5227929540983, 1872.21048901414, 1895.4868333856234, 1933.0743620278388, 1987.3616854155962, 2051.401712085904, 2115.913425091291, 2171.6158074843243, 2209.22784231753, 2221.6289437814494, 2210.340250618501, 2179.0433327091164, 2131.4197599337026, 2071.1511021726847, 2002.6070184063537, 1932.9095240145205, 1869.8687234768877, 1821.2947212731494, 1794.997621882993, 1796.0839700909396, 1818.846071900785, 1854.872673621139, 1895.7525215606233, 1933.0743620278388, 1958.9797703622105, 1967.8216380263405, 1954.5056855136186, 1913.9376333174641, 1841.0232019312798, 1735.210606464688, 1614.1180404922918, 1499.9061922049054, 1414.7357497933558, 1380.7674014484562, 1412.853371717271, 1496.612030571763, 1610.3532843401226, 1732.3870393505647, 1841.0232019312798, 1919.748192428831, 1972.7544872633073, 2009.41107687314, 2039.0869516967841, 2071.1511021726847, 2112.6210655211507, 2161.1085660899125, 2211.8738750085918, 2260.177263406781, 2301.279002414097, 2330.9866156104536, 2347.296636377085, 2348.752850545537, 2333.899043947358, 2301.279002414097, 2250.335364474917, 2184.106179449449, 2106.5283493549473, 2021.5387762086632, 1933.0743620278388, 1843.5067780716506, 1748.9467725668987, 1643.939862982299, 1523.0315667865818, 1380.7674014484562, 1214.1093176483002, 1029.6849989130546, 836.5385619813277, 643.7141235917179, 460.25580048281927, 295.2077093932332, 157.61396706155946, 56.51869022639639, 0.9659956263435976, 0.0, 0.0]

df = pd.DataFrame({'RPM': rpm_values})

# Specify the file path to save the Excel file
file_path = 'indhwy_values.xlsx'

# Save the DataFrame to an Excel file
df.to_excel(file_path, index=False)

print(f"RPM values saved to '{file_path}' successfully.")