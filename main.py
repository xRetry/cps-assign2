#!/usr/bin/env pybricks-micropython

from two_link_arm import TwoLinkArm
from pybricks.parameters import Port
from path_creation import path_line, path_spiral
# from coordinate_affine_transform import transform_coordinates


REFS = [
    [5, 5], # lower right
    [0, 0], # lower left
    [-5, 5], # upper left
]

params = dict(
    lengths=[ 13.7*10**-1, 9*10**-1],
    motor_speed=200,
    ports=[Port.A, Port.B],
    dist_threshold=0.1,
    smp_rate_measure=50, #ms
    smp_rate_target=10,
    calibrate=True
)


def run_measuring():
    arm = TwoLinkArm(**params)
    arm.measure_coordinates()


def run_path_line():
    arm = TwoLinkArm(**params)
    arm.motors[0].reset_angle(0)
    arm.motors[1].reset_angle(0)

    #path = [[1,1]] # , [0.00000000001,1], [-0.5,0.5]
    path = get_line()

    arm.follow_path(path)

def run_path_spiral():
    arm = TwoLinkArm(**params)
    arm.motors[0].reset_angle(0)
    arm.motors[1].reset_angle(0)

    #path = get_spiral()    
    path = path_spiral(100, sum(params["lengths"]), 0.5)

    arm.follow_path(path)


def get_line():
    return [[1.0, 1.0],
            [0.9797979797979798, 1.0],
            [0.9595959595959596, 1.0],
            [0.9393939393939394, 1.0],
            [0.9191919191919192, 1.0],
            [0.898989898989899, 1.0],
            [0.8787878787878788, 1.0],
            [0.8585858585858586, 1.0],
            [0.8383838383838383, 1.0],
            [0.8181818181818181, 1.0],
            [0.797979797979798, 1.0],
            [0.7777777777777778, 1.0],
            [0.7575757575757576, 1.0],
            [0.7373737373737373, 1.0],
            [0.7171717171717171, 1.0],
            [0.696969696969697, 1.0],
            [0.6767676767676767, 1.0],
            [0.6565656565656566, 1.0],
            [0.6363636363636364, 1.0],
            [0.6161616161616161, 1.0],
            [0.595959595959596, 1.0],
            [0.5757575757575757, 1.0],
            [0.5555555555555556, 1.0],
            [0.5353535353535354, 1.0],
            [0.5151515151515151, 1.0],
            [0.4949494949494949, 1.0],
            [0.4747474747474747, 1.0],
            [0.4545454545454546, 1.0],
            [0.43434343434343436, 1.0],
            [0.41414141414141414, 1.0],
            [0.3939393939393939, 1.0],
            [0.3737373737373737, 1.0],
            [0.3535353535353535, 1.0],
            [0.33333333333333337, 1.0],
            [0.31313131313131315, 1.0],
            [0.29292929292929293, 1.0],
            [0.2727272727272727, 1.0],
            [0.2525252525252525, 1.0],
            [0.23232323232323238, 1.0],
            [0.21212121212121215, 1.0],
            [0.19191919191919193, 1.0],
            [0.1717171717171717, 1.0],
            [0.1515151515151515, 1.0],
            [0.13131313131313127, 1.0],
            [0.11111111111111116, 1.0],
            [0.09090909090909094, 1.0],
            [0.07070707070707072, 1.0],
            [0.0505050505050505, 1.0],
            [0.030303030303030276, 1.0],
            [0.010101010101010055, 1.0],
            [-0.010101010101010166, 1.0],
            [-0.030303030303030276, 1.0],
            [-0.05050505050505061, 1.0],
            [-0.07070707070707072, 1.0],
            [-0.09090909090909083, 1.0],
            [-0.11111111111111116, 1.0],
            [-0.13131313131313127, 1.0],
            [-0.1515151515151516, 1.0],
            [-0.1717171717171717, 1.0],
            [-0.19191919191919182, 1.0],
            [-0.21212121212121215, 1.0],
            [-0.23232323232323226, 1.0],
            [-0.2525252525252526, 1.0],
            [-0.2727272727272727, 1.0],
            [-0.29292929292929304, 1.0],
            [-0.31313131313131315, 1.0],
            [-0.33333333333333326, 1.0],
            [-0.3535353535353536, 1.0],
            [-0.3737373737373737, 1.0],
            [-0.39393939393939403, 1.0],
            [-0.41414141414141414, 1.0],
            [-0.43434343434343425, 1.0],
            [-0.4545454545454546, 1.0],
            [-0.4747474747474747, 1.0],
            [-0.49494949494949503, 1.0],
            [-0.5151515151515151, 1.0],
            [-0.5353535353535352, 1.0],
            [-0.5555555555555556, 1.0],
            [-0.5757575757575757, 1.0],
            [-0.595959595959596, 1.0],
            [-0.6161616161616161, 1.0],
            [-0.6363636363636365, 1.0],
            [-0.6565656565656566, 1.0],
            [-0.6767676767676767, 1.0],
            [-0.696969696969697, 1.0],
            [-0.7171717171717171, 1.0],
            [-0.7373737373737375, 1.0],
            [-0.7575757575757576, 1.0],
            [-0.7777777777777777, 1.0],
            [-0.797979797979798, 1.0],
            [-0.8181818181818181, 1.0],
            [-0.8383838383838385, 1.0],
            [-0.8585858585858586, 1.0],
            [-0.8787878787878789, 1.0],
            [-0.898989898989899, 1.0],
            [-0.9191919191919191, 1.0],
            [-0.9393939393939394, 1.0],
            [-0.9595959595959596, 1.0],
            [-0.9797979797979799, 1.0],
            [-1.0, 1.0]]

def get_spiral():
    return [
        [0.375, 0.7503047935520467],
        [0.3897255602515596, 0.8546063380122242],
        [0.4192524179813419, 0.9494791630685875],
        [0.4611841326004773, 1.0329428953876572],
        [0.5129821139084699, 1.1035678839821195],
        [0.5720582017785372, 1.1604640284734886],
        [0.6358594830030253, 1.2032540841497845],
        [0.7019435079399035, 1.2320338253538612],
        [0.7680425501674615, 1.247321615113369],
        [0.8321160265425586, 1.25],
        [0.8923906488800909, 1.2412519323559135],
        [0.9473882999719829, 1.2224941265938265],
        [0.9959420060841874, 1.195309893095203],
        [1.03720070792882, 1.1613835741322287],
        [1.0706238072042646, 1.122438443594976],
        [1.095966683138907, 1.080179638588184],
        [1.1132585321558963, 1.0362433783518825],
        [1.1227739847894807, 0.9921534059966477],
        [1.125, 0.9492852718360435],
        [1.1205995321548223, 0.9088387730891749],
        [1.1103734154253115, 0.871818581519042],
        [1.0952218203341988, 0.8390228348162281],
        [1.076106514414772, 0.8110392443670051],
        [1.0540150104882446, 0.7882480850741036],
        [1.0299275190858506, 0.7708312842618095],
        [1.0047874430480388, 0.7587867171323746],
        [0.9794759689865994, 0.7519467452064041],
        [0.9547911282397288, 0.75],
        [0.931431524671511, 0.752515414221401],
        [0.9099847628930633, 0.7589675335752133],
        [0.8909204621101573, 0.7687621998038623],
        [0.8745876108628066, 0.7812617754151185],
        [0.8612159085744685, 0.795809177943139],
        [0.8509206523786637, 0.8117501017994149],
        [0.8437106626388646, 0.8284529241037508],
        [0.8394986976821699, 0.8453259128780096],
        [0.8381137866280403, 0.8618314775062668],
        [0.8393149073443342, 0.877497318712833],
        [0.8428054525747944, 0.8919244452830811],
        [0.8482479588683911, 0.9047921246987862],
        [0.8552786175639511, 0.9158599227028711],
        [0.8635211420582842, 0.9249670610529692],
        [0.8725996281776338, 0.9320293824436294],
        [0.8821501119854023, 0.9370342563886966],
        [0.8918305992170558, 0.9400337898724143],
        [0.9013294103455162, 0.9411367223589943],
        [0.9103717528981546, 0.9404993872378056],
        [0.9187244962040447, 0.9383161122324503],
        [0.9261991816942565, 0.9348094112194705],
        [0.9326533529845784, 0.9302202909630168],
        [0.9379903333488353, 0.9247989602571898],
        [0.9421576132793547, 0.9187961876990891],
        [0.9451440373787938, 0.91245550959328],
        [0.9469759978727113, 0.9060064430386872],
        [0.9477128518716508, 0.8996588126781426],
        [0.9474417816651876, 0.8935982543477484],
        [0.9462723125032896, 0.887982916210299],
        [0.9443306913620644, 0.8829413389500758],
        [0.9417543140548773, 0.8785714620827453],
        [0.938686367753447, 0.8749406740088899],
        [0.9352708325704778, 0.8720867995089079],
        [0.9316479603608032, 0.8700199001273933],
        [0.927950322317244, 0.8687247503194377],
        [0.9242994901994602, 0.8681638451480318],
        [0.9208033899795898, 0.8682807933981485],
        [0.9175543420525752, 0.8690039527506261],
        [0.9146277795593981, 0.8702501705801209],
        [0.9120816163007511, 0.8719285043767666],
        [0.9099562185423402, 0.8739438090661352],
        [0.9082749209703563, 0.8762000939221077],
        [0.9070450162653981, 0.8786035686413725],
        [0.9062591402316733, 0.8810653158099566],
        [0.9058969700509273, 0.8835035448175996],
        [0.9059271518443917, 0.8858453996985498],
        [0.9063093750646629, 0.8880283099004629],
        [0.9069965149880319, 0.8900008881870745],
        [0.9079367703788681, 0.8917233934285826],
        [0.9090757308662537, 0.8931677876744466],
        [0.9103583173114109, 0.8943174264689242],
        [0.9117305480555451, 0.8951664287733344],
        [0.9131410940374732, 0.8957187780893284],
        [0.9145425959981887, 0.8959872094911785],
        [0.9158927270169797, 0.8959919383886228],
        [0.917154993161636, 0.8957592861215488],
        [0.9182992738385399, 0.895320255140015],
        [0.9193021112992235, 0.8947091027836016],
        [0.9201467655490578, 0.8939619577978586],
        [0.920823056510262, 0.8931155179772311],
        [0.9213270196611134, 0.8922058609681293],
        [0.921660404495444, 0.8912673935598094],
        [0.9218300470501141, 0.8903319579764726],
        [0.9218471484967478, 0.8894281069825378],
        [0.9217264914804204, 0.8885805532198252],
        [0.9214856246285065, 0.8878097922766743],
        [0.921144043581294, 0.8871318936797807],
        [0.9207223941576892, 0.8865584494023825],
        [0.9202417200157421, 0.8860966656676962],
        [0.9197227735509146, 0.8857495808333606],
        [0.9191854049430227, 0.885516389980951],
        [0.9186480403554158, 0.8853928554876215],
        ]

# run_measuring()
run_path_line()
# run_path_spiral()

