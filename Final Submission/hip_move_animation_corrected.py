import pandas as pd 
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

## This Code contains the animation considering the movement of torso (Hip movement) with the corrected step height

# Joint angles calculated from inverse_kinematic.py 
q1 = np.array([0.16987256809594342, 0.1681986723448008, 0.16727823205140013, 0.16728349136954135, 0.1672888419958587, 0.16639345152428353, 0.16549979137902104, 0.16514465538798406, 0.16468418598826773, 0.1637929132790248, 0.1629039301138273, 0.16250133842291103, 0.16207169073138217, 0.16122098521842143, 0.16033235860686257, 0.1599224660600329, 0.15887923535111415, 0.15673949324782077, 0.15457289865714308, 0.1524853500866341, 0.15047765554541015, 0.149114826850689, 0.14790249831745395, 0.14572525505505807, 0.14336849240279514, 0.14169311583966793, 0.1400221522330396, 0.1378224166187365, 0.1355244839773011, 0.1336069695234552, 0.13154196215355562, 0.12808689519737237, 0.12473306814547791, 0.1225449014540736, 0.12050877273857497, 0.11830857659369043, 0.1161420548277814, 0.11487741100744708, 0.11357792799554955, 0.11121424685900716, 0.10884240655056376, 0.10687664262638008, 0.10505367285961675, 0.10362007426724884, 0.10216556909394292, 0.10000171791420498, 0.09776632780072148, 0.09595029231468577, 0.0941906298070938, 0.0923760068934969, 0.09045774702847087, 0.08791936319273819, 0.0855933972220877, 0.08428582487256242, 0.0829507838671717, 0.08140550675187619, 0.07974940018470922, 0.0779218036558258, 0.07611030032227406, 0.07425306229128492, 0.07234744852378117, 0.07043544542186941, 0.06852111470318079, 0.066795189258956, 0.06529287396216454, 0.06384829913414158, 0.062474982429287396, 0.0612266592591022, 0.060082666992081535, 0.059098268797074915, 0.05822884940329365, 0.057547500990883016, 0.05722837503673106, 0.05739506598524691, 0.057607863645041446, 0.05734396313236978, 0.0568993501787487, 0.05663784091332169, 0.0568583681988426, 0.05779505809534524, 0.05888867950325305, 0.059202290848779926, 0.059636012934429994, 0.06110378675500816, 0.06309392072850484, 0.06541505439298256, 0.06781852335023308, 0.06971359400950261, 0.07197312202256856, 0.07500018919693896, 0.07809772274064875, 0.08132131111513163, 0.08469204985445566, 0.08821771261185773, 0.09181540057939208, 0.09558222723465581, 0.09955475490230148, 0.10396717731289629, 0.1084792620774655, 0.11265842814423088, 0.11687306096311345, 0.12114656523497724, 0.12577551804447018, 0.13093122185642625, 0.13617508632580932, 0.14135929697356753, 0.14666724797861175, 0.15213477143231469, 0.15758040716096944, 0.16248228876969162, 0.1673066840985945, 0.17296372657761627, 0.17850004155007104, 0.18347579376832446, 0.18846277113790344, 0.19347719594404422, 0.19860264172221698, 0.2044513236434985, 0.2109095665529972, 0.2176905809255838, 0.22428232372795898, 0.23038136866860692, 0.23643470718926296, 0.24231697527363538, 0.2484681617237734, 0.2552087080729454, 0.2620716455268707, 0.26843230794311246, 0.2747097927185268, 0.28135725095520414, 0.28835606745381914, 0.294982439113436, 0.30143051002545596, 0.30883968877439116, 0.3160656644682397, 0.32280872153063533, 0.3296351486648418, 0.33633562072386725, 0.34304943755263906, 0.3498421284957044, 0.3567519461761478, 0.3637426923271525, 0.37073070115847073, 0.377718140216873, 0.3847029897105292, 0.3924753373687344, 0.4004922550130209, 0.40751979657112425, 0.4145859623555993, 0.4225649559799861, 0.43063081313090557, 0.43828479819678035, 0.4454351042899447, 0.4521740148005021, 0.45892654836121805, 0.46582043102310555, 0.472429529848048, 0.47853997737750975, 0.48464949604196583, 0.4908720617270439, 0.49703898086865084, 0.5026690739011771, 0.5080094283967143, 0.5136073192967742, 0.5191695252091064, 0.5244411931218274, 0.5295192127577005, 0.5345597681062908, 0.5394413933044475, 0.5432999363547661, 0.5468216905959067, 0.5509483197727991, 0.5553695869700694, 0.5606621751831213, 0.5657120437570667, 0.5696236243496053, 0.5733631320953393, 0.5772260956717228, 0.5808414350412494, 0.5837793683213488, 0.5864419609222102, 0.5891603335288917, 0.5917392114943518, 0.5934973274081168, 0.5950867852975761, 0.5975639588272685, 0.5998678864272287, 0.6014521196607014, 0.6030721010452504, 0.6049612556430022, 0.6069523582557435, 0.6089306399599093, 0.6112106437061045, 0.6140011547672857, 0.6168997514816561, 0.6197939973311013, 0.6226504140132665, 0.6246558967927034, 0.6265635331806025, 0.6292923522265064, 0.6322352236248643, 0.635435778269662, 0.63839054347458, 0.6406996269325673, 0.6427509856568161, 0.6447852935390943, 0.6468440436155292, 0.6489244776966572, 0.6510542874561078, 0.6530816818319842, 0.6549350548583397, 0.6565736227064891, 0.6580711814238109, 0.6596790806968882, 0.6609647697633974, 0.6612073988946405, 0.6608783838409573, 0.6605610629779988, 0.6602312256703422, 0.6598047709573724, 0.6598722686615237, 0.6603636205152077, 0.6605285905893119, 0.6605729032383465, 0.6604520247764334, 0.659984283573982, 0.6596839023813637, 0.6597596745989527, 0.6598473337840163, 0.6599278675776936, 0.6599510868793383, 0.6597557496459552, 0.6589702457036082, 0.6572349295376447, 0.6554038783302831, 0.654097086401036, 0.6528268994439104, 0.6515326140345001, 0.6499498106474342, 0.6481331925558242, 0.6462522558701935, 0.6442440215844307, 0.6422586278295432, 0.6403355037225, 0.6385184693551709, 0.6372165368966578, 0.6360918527889678, 0.6344174230279507, 0.6327412608370093, 0.6324053906628833, 0.6319768795823042, 0.629489228683694, 0.6270496456273462, 0.6255639099056978, 0.6241013332413898, 0.622661765964063, 0.6212436068007303, 0.6199023687956609, 0.6183282785698987, 0.6165071073854085, 0.6148798766238108, 0.6135085152640092, 0.6121870679068389, 0.6104878776740343, 0.6090280947664586, 0.6074898710674281, 0.6058279068699035, 0.6050260397918312, 0.6043870191968064, 0.6029166724776227, 0.6016307428558141, 0.6007772818622044, 0.6000226233043024, 0.6000374982400989, 0.599945764829116, 0.599482291829889, 0.5990197329155689, 0.5988460449083954, 0.5987618435390811, 0.5984210259538707, 0.5979467106170966])
q2 = np.array([0.6946188074451891, 0.6945769310330472, 0.6945541609732078, 0.694554296400085, 0.6945544338276866, 0.6945314375494864, 0.6945083945583398, 0.6945008229066477, 0.6944918337139323, 0.6944687005425356, 0.6944455326632475, 0.6944382973353306, 0.6944313746501057, 0.6944087803719092, 0.6943854863997898, 0.694381009139331, 0.6943685718537057, 0.6943372089519545, 0.6943119051468324, 0.6942955928076661, 0.6942881999373427, 0.6943054527654015, 0.6943339061638241, 0.6943559689398442, 0.694396421191265, 0.6944678702947314, 0.6945521491677693, 0.6946275239268048, 0.6947167245700383, 0.6948587327154946, 0.695031059905964, 0.6951802561582828, 0.6953511282029803, 0.6955757611703911, 0.6958244145397282, 0.6961299124693326, 0.6964795851232152, 0.6968035266397269, 0.6971729470334329, 0.6976449173053305, 0.6981805561544051, 0.6987819919730335, 0.6994171099359296, 0.7000235777618757, 0.7006583928482459, 0.7013612266719417, 0.7021012965761735, 0.7028876306654642, 0.7037328088394998, 0.7047051846617385, 0.7057740257325621, 0.7068707639536618, 0.7080195973939494, 0.7091280122466127, 0.7103033470129418, 0.7116591139354546, 0.7130662786961549, 0.7145179080779528, 0.7160235504717326, 0.7176267269225012, 0.7193325219557407, 0.7211191313826358, 0.7229851182164801, 0.7249484864476753, 0.7270512833550011, 0.7293865721790547, 0.7319690893671973, 0.7347331583763272, 0.7376905584102076, 0.7408411628118784, 0.7442893343285928, 0.7480601525958093, 0.7520362101512111, 0.7561888175235936, 0.7604952873147394, 0.7649368912152791, 0.7698119398022455, 0.7751937814170775, 0.7807705263298651, 0.7864576966770974, 0.7923287776795093, 0.7987929409820915, 0.80581015005904, 0.8130967149269204, 0.8205247992717353, 0.8278855163533325, 0.8353799545468683, 0.8431168595871524, 0.8511372295549148, 0.8595881944539195, 0.8682338689279485, 0.8769127605052103, 0.885536315261463, 0.8940449911328754, 0.902660362702654, 0.9114389393186393, 0.9203396680998592, 0.9293723230790695, 0.9381859425220775, 0.9463304482229247, 0.9543806922225408, 0.9627486282552846, 0.9712030001774784, 0.9797462785333133, 0.9883623269760449, 0.9971465529603643, 1.0055752748058875, 1.0132542669927933, 1.020818067495733, 1.0286141082367213, 1.0363179527149387, 1.0437042855356982, 1.0510374452743954, 1.0584750552644338, 1.0659501148278925, 1.0735356519638324, 1.0808623883909636, 1.0878568185348467, 1.0948925151606574, 1.1019663828317534, 1.1090611823074044, 1.1156251913300612, 1.1224321163080941, 1.130155880965873, 1.138016263986879, 1.1459216184848642, 1.1539373629362764, 1.1651415363246929, 1.1772998680137787, 1.1868380608402114, 1.193624935891755, 1.1980465076939089, 1.2024566328636948, 1.2068998621175888, 1.2113210272532313, 1.2142213300442888, 1.2164069990086848, 1.2196695033616707, 1.2228478202296236, 1.2253376273315777, 1.2268218742570651, 1.2275878730081793, 1.2283480976702648, 1.2291153453516768, 1.2298754067714708, 1.2296844756994831, 1.229023644849225, 1.2293189121876464, 1.2293507408569804, 1.2280446948773716, 1.2261395825788226, 1.2242237389824009, 1.2216610051576637, 1.2177078022754348, 1.2134603567154163, 1.2100460254052199, 1.206412571618234, 1.201598886224396, 1.1960112546947412, 1.190032403814854, 1.184066224411181, 1.1786474855455462, 1.173274945788393, 1.1671081883149133, 1.1598055149643482, 1.1509103543077845, 1.1420787169385853, 1.1342996418881675, 1.1265965864046867, 1.118986248529058, 1.1114120346215837, 1.1035228411400635, 1.094357779985113, 1.083270667343442, 1.0710704036523935, 1.0586021548408382, 1.0462533311892073, 1.0340648920537503, 1.022946029519968, 1.0132956635828558, 1.0040418021444006, 0.994867101767239, 0.9857668087861933, 0.9767477692720069, 0.9678117064663686, 0.9591320509848803, 0.9507963749402494, 0.9426522728520933, 0.934593152998777, 0.9265741588289849, 0.9186383992200253, 0.9111756920828439, 0.9036996713180905, 0.8958392107160086, 0.8880696975513214, 0.880374862403164, 0.8728633293355269, 0.8655815797745287, 0.8584042309063397, 0.8513218301652017, 0.8443433116961986, 0.8378138057100312, 0.8315846550806847, 0.825312032896026, 0.8191607127266721, 0.8131739138453197, 0.8073654366473821, 0.8017407814647479, 0.7962440293059179, 0.7907537585145089, 0.7854563204278583, 0.7807435087849894, 0.7764111251782408, 0.7721463447461804, 0.768013550101195, 0.764058971772372, 0.7602880143036871, 0.7567326003072841, 0.753340966871888, 0.7500807337882613, 0.7470303941948497, 0.7444156971272774, 0.742261177461383, 0.7402970473330621, 0.7385575096122783, 0.7370089201227742, 0.735480792812954, 0.7339670805084193, 0.7324779040267714, 0.7310340675995779, 0.7296762472527073, 0.7285209828408588, 0.7274485515493511, 0.7263343224780756, 0.7251518235349375, 0.7238632888263244, 0.7225702186171321, 0.7212958018156758, 0.7200424916243156, 0.7187039476862158, 0.7174329859095576, 0.7163351472418384, 0.7152639990920971, 0.7142169786440463, 0.7131941360041513, 0.7121847001967091, 0.7111981723930317, 0.7102450422126415, 0.7093025092804651, 0.708245586556189, 0.7072032088848803, 0.7063958180551958, 0.7055849306963152, 0.7046825815212879, 0.7038180800940459, 0.7029916689198876, 0.7022035035334391, 0.7014501591028486, 0.7007363064940474, 0.6999953540645577, 0.6992277768407512, 0.6984849333624632, 0.6978027260559172, 0.6972723441559714, 0.6967457133572528, 0.6961975206891043, 0.6957344050579802, 0.6953300871759737, 0.6950035723705745, 0.6947816502979548, 0.6946381007478294, 0.6945957485174892, 0.6946050343115979, 0.6945968627542424, 0.6945923217801062, 0.6945983728143283, 0.6946051961568495, 0.6946025931341139, 0.6945996316809602, 0.694606812927517, 0.6946188074451891])
t = np.linspace(0,2.8,280)

#link lengths:
#Given in the doc
ground_height =92.4 #Height from hip to ground (cm)

l1 = 43.2 #Thigh Length : (cm)
l2 = 40.6 #Knee Length : (cm)

ankle_height = 8.6 # (cm)
foot_forward_len = 19.5
foot_backward_len = 26.2 - 19.5

#Plotting the figure for geberating frames
fig,ax = plt.subplots()

# Apending the coordinates for different links
hip_positions = []
l1_end_positions = []
l2_end_positions = []
l3_end_positions = []
foot_front_ends = []
foot_back_ends = []


for t1,t2,t3 in zip(q1,q2,t):
    hip_position = 2.5*np.sin(((np.pi/2.8)*t3))

    l1_end_position_x = l1*np.cos(t1 + np.pi + np.pi/2)
    l1_end_position_y = l1*np.sin(t1+np.pi + np.pi/2) +hip_position
    l1_end_position = np.array([l1_end_position_x,l1_end_position_y])
    l2_end_position_x = l1_end_position_x + l2*np.cos(-np.pi + (t1 - t2) + np.pi/2)
    l2_end_position_y = l1_end_position_y + l2*np.sin(-np.pi + (t1 - t2) + np.pi/2)
    l2_end_position = np.array([l2_end_position_x,l2_end_position_y])

    l3_end_position_x = l1_end_position_x + (l2 + ankle_height)*(np.cos(-np.pi + (t1 - t2) + np.pi/2))
    l3_end_position_y = l1_end_position_y + (l2 + ankle_height)*(np.sin(-np.pi + (t1 - t2) + np.pi/2))
    l3_end_position = np.array([l3_end_position_x,l3_end_position_y])
    
    foot_front_end_x = l3_end_position_x + foot_backward_len*(np.cos(-3*np.pi/2 + (t1 - t2) + np.pi/2))
    foot_front_end_y = l3_end_position_y + foot_backward_len*(np.sin(-3*np.pi/2 + (t1 - t2) + np.pi/2))
    foot_front_end = np.array([foot_front_end_x,foot_front_end_y])

    foot_back_end_x = l3_end_position_x - foot_forward_len*(np.cos(-3*np.pi/2 + (t1 - t2) + np.pi/2))
    foot_back_end_y = l3_end_position_y - foot_forward_len*(np.sin(-3*np.pi/2 + (t1 - t2) + np.pi/2))
    foot_back_end = np.array([foot_back_end_x,foot_back_end_y])

    hip_positions.append(hip_position)
    l1_end_positions.append(l1_end_position)
    l2_end_positions.append(l2_end_position)
    l3_end_positions.append(l3_end_position)
    foot_front_ends.append(foot_front_end)
    foot_back_ends.append(foot_back_end)

# Various attributes for the animation
gait_x = [pos[0] for pos in l2_end_positions]
gait_y = [pos[1] for pos in l2_end_positions]
ax.scatter(gait_x, gait_y, s = 0.6, alpha = 0.5)

l1_end_circle = plt.Circle([0,0], radius=4, fc='#0077b6', alpha = 0.5)
l2_end_circle = plt.Circle([0,0], radius=4, fc='g',  alpha = 0.5)
l1_line = plt.Line2D([], [], color = "#0077b6" , lw=2.5)
l2_line = plt.Line2D([], [], color = '#0077b6', lw=2.5)
foot_line = plt.Line2D([], [], color = '#0077b6', lw=2.5)
time_text = ax.text(0.02, 0.95, 'time (seconds): 0.00', transform = ax.transAxes)


ax.axhline(-(l1 + l2), alpha = 0)
ax.axhline((l1 + l2), alpha = 0)
ax.axvline((l1 + l2), alpha = 0)
ax.axvline(-(l1 + l2), alpha = 0)
ax.axhline(-ground_height, color = 'g') # Ground Line

ax.add_line(l2_line)
ax.add_patch(foot_line)
ax.add_line(l1_line)
ax.add_patch(l1_end_circle)
ax.add_patch(l2_end_circle)

plt.axis('square')

coll_points = []
def animate(i):
    t_ = t[i]
     
    l1_line.set_data((0,l1_end_positions[i][0]),(hip_positions[i], l1_end_positions[i][1]))
    l2_line.set_data((l1_end_positions[i][0],l3_end_positions[i][0]),(l1_end_positions[i][1],l3_end_positions[i][1]))
    foot_line.set_data((foot_back_ends[i][0], foot_front_ends[i][0]),(foot_back_ends[i][1], foot_front_ends[i][1]))

    l1_end_circle.set_center(l1_end_positions[i])
    l2_end_circle.set_center(l2_end_positions[i])
      
    time_text.set_text('time (seconds): %.2f' % t_)
    
    if (foot_front_ends[i][1] < -ground_height) or (foot_back_ends[i][1] < -ground_height):
        foot_line.set_color('r')
        coll_points.append(i)
    else:
        foot_line.set_color('#0077b6')

    return l1_line, l2_line, l1_end_circle, l2_end_circle, foot_line, time_text

anim = animation.FuncAnimation(fig, animate,
                            frames = 280,
                            interval = 2.8,
                            blit = True)

ax.set_title('Moving Hip Joint: After Increment')
ax.grid()
plt.show()

# Saving the GIF
writergif = animation.PillowWriter(fps=100) 
anim.save(r'C:\\Users\\videh\\OneDrive\Documents\Sem 5\\ME 639\\Coffee Bots\Workspace\\gifs\\hip_move_after.gif', writer = writergif)