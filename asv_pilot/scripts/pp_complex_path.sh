#!/usr/bin/env bash

rosservice call /path/control """
command: 'path'
points:
- values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- values: [0.8970545856386584, 0.046784867020784215, 0.0, 0.0, 0.0, 0.05210665903471886]
- values: [1.699737760581377, 0.18289857119516925, 0.0, 0.0, 0.0, 0.1679754971639117]
- values: [2.417433915026165, 0.40197976719120415, 0.0, 0.0, 0.0, 0.2962719868853314]
- values: [3.0595274391710325, 0.6976671096769375, 0.0, 0.0, 0.0, 0.43155554744874497]
- values: [3.6354027232139896, 1.0635992533204186, 0.0, 0.0, 0.0, 0.5660689987431853]
- values: [4.154444157353043, 1.493414852789696, 0.0, 0.0, 0.0, 0.6916388064368612]
- values: [4.626036131786205, 1.980752562752818, 0.0, 0.0, 0.0, 0.8018168027329492]
- values: [5.059563036711483, 2.5192510378778348, 0.0, 0.0, 0.0, 0.8929738816198414]
- values: [5.464409262326891, 3.1025489328327938, 0.0, 0.0, 0.0, 0.9640651949581379]
- values: [5.849959198830431, 3.724284902285745, 0.0, 0.0, 0.0, 1.0157150541309676]
- values: [6.225597236420119, 4.378097600904738, 0.0, 0.0, 0.0, 1.049311854413369]
- values: [6.600707765293962, 5.057625683357819, 0.0, 0.0, 0.0, 1.066406466161969]
- values: [6.984675175649968, 5.756507804313037, 0.0, 0.0, 0.0, 1.0684121561650266]
- values: [7.386883857686151, 6.468382618438445, 0.0, 0.0, 0.0, 1.0565104115827884]
- values: [7.816718201600516, 7.186888780402089, 0.0, 0.0, 0.0, 1.031676850848109]
- values: [8.283562597591072, 7.905664944872017, 0.0, 0.0, 0.0, 0.9947733876410657]
- values: [8.796801435855832, 8.618349766516278, 0.0, 0.0, 0.0, 0.9466755423081483]
- values: [9.365819106592804, 9.318581900002922, 0.0, 0.0, 0.0, 0.8884117648086237]
- values: [10.0, 10.0, 0.0, 0.0, 0.0, 0.8212882203698031]
- values: [10.0, 9.999999999999998, 0.0, 0.0, 0.0, 0.7853981633974483]
- values: [9.502182481629351, 10.546079804276687, 0.0, 0.0, 0.0, 0.692998379468337]
- values: [9.05687445270596, 11.135762934111646, 0.0, 0.0, 0.0, 0.6005985955392248]
- values: [8.667875124321423, 11.764018420957528, 0.0, 0.0, 0.0, 0.5081988116101135]
- values: [8.338503301588213, 12.425486210517217, 0.0, 0.0, 0.0, 0.4157990276810022]
- values: [8.071569068764653, 13.114522892837169, 0.0, 0.0, 0.0, 0.32339924375189]
- values: [7.869349814606837, 13.825249849891307, 0.0, 0.0, 0.0, 0.2309994598227787]
- values: [7.733570802490739, 14.55160340987732, 0.0, 0.0, 0.0, 0.1385996758936674]
- values: [7.6653904510734, 15.287386580326459, 0.0, 0.0, 0.0, 0.046199891964556095]
- values: [7.6653904510734, 16.026321918658297, 0.0, 0.0, 0.0, -0.046199891964556095]
- values: [7.733570802490738, 16.762105089107436, 0.0, 0.0, 0.0, -0.1385996758936674]
- values: [7.869349814606836, 17.48845864909345, 0.0, 0.0, 0.0, -0.2309994598227787]
- values: [8.071569068764651, 18.199185606147587, 0.0, 0.0, 0.0, -0.32339924375189]
- values: [8.338503301588212, 18.88822228846754, 0.0, 0.0, 0.0, -0.4157990276810022]
- values: [8.667875124321423, 19.54969007802723, 0.0, 0.0, 0.0, -0.5081988116101135]
- values: [9.056874452705959, 20.177945564873113, 0.0, 0.0, 0.0, -0.6005985955392248]
- values: [9.502182481629347, 20.767628694708073, 0.0, 0.0, 0.0, -0.692998379468337]
- values: [9.999999999999998, 21.31370849898476, 0.0, 0.0, 0.0, -0.7853981633974483]
- values: [9.999999999999998, 21.31370849898476, 0.0, 0.0, 0.0, 2.356194490192345]
- values: [9.999999999999998, 21.31370849898476, 0.0, 0.0, 0.0, 0.0]
- values: [9.305675067916107, 21.782662208300522, 0.0, 0.0, 0.0, 2.5475616643337613]
- values: [8.566512752963309, 21.873439573274748, 0.0, 0.0, 0.0, 3.0193934237863593]
- values: [7.788559872665008, 21.62158685387278, 0.0, 0.0, 0.0, -2.8285028346886723]
- values: [6.977863244544613, 21.06265031005998, 0.0, 0.0, 0.0, -2.53798090850601]
- values: [6.14046968612552, 20.23217620180167, 0.0, 0.0, 0.0, -2.360343146948084]
- values: [5.2824260149311275, 19.165710789063198, 0.0, 0.0, 0.0, -2.248316279510498]
- values: [4.409779048484841, 17.898800331809916, 0.0, 0.0, 0.0, -2.1739653838504207]
- values: [3.5285756043100673, 16.46699109000716, 0.0, 0.0, 0.0, -2.1224969327155163]
- values: [2.6448624999302024, 14.905829323620283, 0.0, 0.0, 0.0, -2.0858869148690933]
- values: [1.7646865528686464, 13.250861292614612, 0.0, 0.0, 0.0, -2.059589242418438]
- values: [0.8940945806488052, 11.537633256955502, 0.0, 0.0, 0.0, -2.0409496844136195]
- values: [0.03913340079408156, 9.8016914766083, 0.0, 0.0, 0.0, -2.028430522196066]
- values: [-0.7941501691721248, 8.078582211538343, 0.0, 0.0, 0.0, -2.021232390605861]
- values: [-1.5997093117264143, 6.403851721710972, 0.0, 0.0, 0.0, -2.0191354093443024]
- values: [-2.371497209345382, 4.8130462670915355, 0.0, 0.0, 0.0, -2.0224979078358265]
- values: [-3.103467044505625, 3.341712107645378, 0.0, 0.0, 0.0, -2.032431634492909]
- values: [-3.789571999683743, 2.0253955033378417, 0.0, 0.0, 0.0, -2.051284077918726]
- values: [-4.423765257356336, 0.8996427141342667, 0.0, 0.0, 0.0, -2.0838315357277533]
- values: [-5.0, 0.0, 0.0, 0.0, 0.0, -2.1404748549654653]
- values: [-5.000000000000001, 0.0, 0.0, 0.0, 0.0, -2.356194490192345]
- values: [-4.50218248162935, -0.5460798042766895, 0.0, 0.0, 0.0, -2.448594274121456]
- values: [-4.056874452705962, -1.1357629341116446, 0.0, 0.0, 0.0, -2.540994058050568]
- values: [-3.667875124321424, -1.7640184209575303, 0.0, 0.0, 0.0, -2.6333938419796796]
- values: [-3.3385033015882124, -2.4254862105172204, 0.0, 0.0, 0.0, -2.725793625908791]
- values: [-3.0715690687646537, -3.1145228928371664, 0.0, 0.0, 0.0, -2.8181934098379022]
- values: [-2.8693498146068377, -3.8252498498913066, 0.0, 0.0, 0.0, -2.910593193767014]
- values: [-2.733570802490738, -4.551603409877323, 0.0, 0.0, 0.0, -3.0029929776961257]
- values: [-2.6653904510734003, -5.287386580326464, 0.0, 0.0, 0.0, -3.095392761625237]
- values: [-2.6653904510734003, -6.026321918658296, 0.0, 0.0, 0.0, 3.095392761625238]
- values: [-2.733570802490738, -6.762105089107439, 0.0, 0.0, 0.0, 3.0029929776961257]
- values: [-2.869349814606837, -7.488458649093454, 0.0, 0.0, 0.0, 2.9105931937670144]
- values: [-3.071569068764653, -8.199185606147594, 0.0, 0.0, 0.0, 2.818193409837903]
- values: [-3.3385033015882115, -8.88822228846754, 0.0, 0.0, 0.0, 2.725793625908791]
- values: [-3.667875124321423, -9.549690078027231, 0.0, 0.0, 0.0, 2.6333938419796796]
- values: [-4.056874452705961, -10.177945564873117, 0.0, 0.0, 0.0, 2.5409940580505683]
- values: [-4.502182481629349, -10.767628694708073, 0.0, 0.0, 0.0, 2.448594274121456]
- values: [-4.999999999999999, -11.313708498984761, 0.0, 0.0, 0.0, 2.356194490192345]
- values: [-4.999999999999999, -11.313708498984761, 0.0, 0.0, 0.0, -0.7853981633974483]
- values: [-4.999999999999999, -11.313708498984761, 0.0, 0.0, 0.0, 0.0]
- values: [-4.322026232459762, -11.882638088948646, 0.0, 0.0, 0.0, -0.6981680977532083]
- values: [-3.6282838190171214, -12.260359843842632, 0.0, 0.0, 0.0, -0.49858742108937604]
- values: [-2.91936918901426, -12.462945145595162, 0.0, 0.0, 0.0, -0.27834956093678276]
- values: [-2.195878771793364, -12.50646537613467, 0.0, 0.0, 0.0, -0.06008075842170418]
- values: [-1.458408996696613, -12.406991917389588, 0.0, 0.0, 0.0, 0.13407558233967892]
- values: [-0.7075562930661912, -12.180596151288348, 0.0, 0.0, 0.0, 0.29284906859608]
- values: [0.05608290975571828, -11.843349459759388, 0.0, 0.0, 0.0, 0.41587244873219975]
- values: [0.8319121824269313, -11.411323224731145, 0.0, 0.0, 0.0, 0.508092724225087]
- values: [1.619335095605266, -10.900588828132056, 0.0, 0.0, 0.0, 0.5754010412469925]
- values: [2.417755219948539, -10.327217651890543, 0.0, 0.0, 0.0, 0.6227918325279089]
- values: [3.226576126114568, -9.707281077935052, 0.0, 0.0, 0.0, 0.6539585491886509]
- values: [4.045201384761167, -9.05685048819402, 0.0, 0.0, 0.0, 0.6714028528902283]
- values: [4.8730345665461545, -8.391997264595872, 0.0, 0.0, 0.0, 0.6766433067651728]
- values: [5.70947924212735, -7.728792789069049, 0.0, 0.0, 0.0, 0.6703874737536555]
- values: [6.553938982162569, -7.083308443541982, 0.0, 0.0, 0.0, 0.6526381322630368]
- values: [7.4058173573096235, -6.471615609943111, 0.0, 0.0, 0.0, 0.6227388316885258]
- values: [8.264517938226335, -5.909785670200867, 0.0, 0.0, 0.0, 0.5793775745445917]
- values: [9.129444295570522, -5.413890006243684, 0.0, 0.0, 0.0, 0.5205847947301229]
- values: [10.0, -5.0, 0.0, 0.0, 0.0, 0.4438007193141674]
- values: [10.0, -4.999999999999999, 0.0, 0.0, 0.0, -0.7853981633974483]
- values: [10.54607980427669, -4.502182481629348, 0.0, 0.0, 0.0, -0.8777979473265596]
- values: [11.135762934111645, -4.056874452705961, 0.0, 0.0, 0.0, -0.9701977312556713]
- values: [11.76401842095753, -3.667875124321423, 0.0, 0.0, 0.0, -1.062597515184783]
- values: [12.42548621051722, -3.3385033015882115, 0.0, 0.0, 0.0, -1.1549972991138946]
- values: [13.114522892837167, -3.071569068764653, 0.0, 0.0, 0.0, -1.2473970830430061]
- values: [13.825249849891307, -2.869349814606837, 0.0, 0.0, 0.0, -1.3397968669721176]
- values: [14.551603409877323, -2.733570802490738, 0.0, 0.0, 0.0, -1.4321966509012292]
- values: [15.287386580326464, -2.6653904510734003, 0.0, 0.0, 0.0, -1.524596434830341]
- values: [16.026321918658297, -2.6653904510734003, 0.0, 0.0, 0.0, -1.6169962187594522]
- values: [16.762105089107436, -2.733570802490738, 0.0, 0.0, 0.0, -1.709396002688564]
- values: [17.488458649093452, -2.869349814606837, 0.0, 0.0, 0.0, -1.8017957866176755]
- values: [18.199185606147594, -3.071569068764653, 0.0, 0.0, 0.0, -1.894195570546787]
- values: [18.88822228846754, -3.3385033015882115, 0.0, 0.0, 0.0, -1.9865953544758985]
- values: [19.54969007802723, -3.667875124321424, 0.0, 0.0, 0.0, -2.07899513840501]
- values: [20.177945564873113, -4.056874452705961, 0.0, 0.0, 0.0, -2.171394922334122]
- values: [20.76762869470807, -4.502182481629348, 0.0, 0.0, 0.0, -2.2637947062632335]
- values: [21.31370849898476, -4.999999999999999, 0.0, 0.0, 0.0, -2.356194490192345]
- values: [21.31370849898476, -4.999999999999999, 0.0, 0.0, 0.0, 0.7853981633974483]
options:
- key: 'mode'
  value: 'simple'
- key: 'timeout'
  value: '3000'
"""
