"""Генератор открытой тестовой платы для автотрассировщика (только стандартные библиотеки KiCad).

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" tools/make_demo_board.py example/demo_board.kicad_pcb

Плата 100×70 мм, 4 слоя (F / In1 = GND / In2 / B). Что на ней проверяется:
  * fanout микросхем с шагом 0.5 мм: MCU в LQFP-48 и драйвер светодиодов в QFN-32 с открытой площадкой
    (переходы под площадкой);
  * USB-C: дифпара USB_D+/USB_D−, CC-резисторы, питание VBUS;
  * матрица кнопок 2×4 с диодами: строки ROW0..1 и столбцы COL0..3 — цепочкой;
  * шина LED_K0..7 от драйвера к восьми светодиодам — параллельные трассы;
  * питание: VBUS (USB → LDO, драйвер, светодиоды, выход на разъём) и +3V3 от LDO;
  * развязывающие конденсаторы у каждого вывода питания — короткие отводы на плоскость GND.
"""
import os, sys
import pcbnew

LIB = os.environ.get('KICAD_FOOTPRINTS', r'C:\Program Files\KiCad\10.0\share\kicad\footprints')
X0, Y0, X1, Y1 = 100.0, 100.0, 200.0, 170.0


def mm(v): return pcbnew.FromMM(v)


class Builder:
    def __init__(self):
        self.b = pcbnew.BOARD()
        self.b.SetCopperLayerCount(4)
        self.nets = {}

    def net(self, name):
        if name not in self.nets:
            n = pcbnew.NETINFO_ITEM(self.b, name); self.b.Add(n); self.nets[name] = n
        return self.nets[name]

    def fp(self, lib, name, ref, x, y, rot=0.0, value='', pins=None):
        m = pcbnew.FootprintLoad(os.path.join(LIB, lib + '.pretty'), name)
        if m is None: raise SystemExit('нет футпринта %s:%s' % (lib, name))
        m.SetReference(ref); m.SetValue(value or name)
        m.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        m.SetOrientationDegrees(rot)
        self.b.Add(m)
        for p in m.Pads():
            n = (pins or {}).get(p.GetNumber())
            if n: p.SetNet(self.net(n))
        return m

    def edge(self):
        pts = [(X0, Y0), (X1, Y0), (X1, Y1), (X0, Y1)]
        for a, b in zip(pts, pts[1:] + pts[:1]):
            s = pcbnew.PCB_SHAPE(self.b); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1]))); s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); self.b.Add(s)

    def gnd_plane(self):
        z = pcbnew.ZONE(self.b); z.SetLayer(pcbnew.In1_Cu); z.SetNet(self.net('GND'))
        o = z.Outline(); o.NewOutline()
        for (x, y) in [(X0 + 0.5, Y0 + 0.5), (X1 - 0.5, Y0 + 0.5), (X1 - 0.5, Y1 - 0.5), (X0 + 0.5, Y1 - 0.5)]:
            o.Append(mm(x), mm(y))
        z.SetLocalClearance(mm(0.25)); z.SetMinThickness(mm(0.2))
        z.SetThermalReliefGap(mm(0.5)); z.SetThermalReliefSpokeWidth(mm(0.5))
        self.b.Add(z)


def build(path):
    B = Builder()
    B.edge()
    # --- USB-C у верхнего края, гнездом наружу
    B.fp('Connector_USB', 'USB_C_Receptacle_HRO_TYPE-C-31-M-12', 'J1', 116, 104.5, 180, 'USB-C',
         {'A1': 'GND', 'B12': 'GND', 'A12': 'GND', 'B1': 'GND', 'A4': 'VBUS', 'B9': 'VBUS', 'A9': 'VBUS', 'B4': 'VBUS',
          'A5': 'CC1', 'B5': 'CC2', 'A6': 'USB_D+', 'B6': 'USB_D+', 'A7': 'USB_D-', 'B7': 'USB_D-', 'SH': 'GND'})
    B.fp('Resistor_SMD', 'R_0603_1608Metric', 'R1', 109, 111, 90, '5.1k', {'1': 'CC1', '2': 'GND'})
    B.fp('Resistor_SMD', 'R_0603_1608Metric', 'R2', 123, 111, 90, '5.1k', {'1': 'CC2', '2': 'GND'})
    B.fp('Capacitor_SMD', 'C_0603_1608Metric', 'C1', 126, 104, 0, '10u', {'1': 'VBUS', '2': 'GND'})
    # --- LDO 3.3 В
    B.fp('Package_TO_SOT_SMD', 'SOT-23-5', 'U3', 132, 106, 0, 'LDO 3V3',
         {'1': 'VBUS', '2': 'GND', '3': 'VBUS', '5': '+3V3'})
    B.fp('Capacitor_SMD', 'C_0603_1608Metric', 'C2', 128, 109, 90, '1u', {'1': 'VBUS', '2': 'GND'})
    B.fp('Capacitor_SMD', 'C_0603_1608Metric', 'C3', 136, 109, 90, '1u', {'1': '+3V3', '2': 'GND'})
    # --- MCU LQFP-48
    mcu = {'1': '+3V3', '7': 'NRST', '8': 'GND', '9': '+3V3', '10': 'ROW0', '11': 'ROW1',
           '12': 'COL0', '13': 'COL1', '14': 'COL2', '15': 'COL3', '23': 'GND', '24': '+3V3',
           '32': 'USB_D-', '33': 'USB_D+', '34': 'SWDIO', '35': 'GND', '36': '+3V3', '37': 'SWCLK',
           '42': 'I2C_SCL', '43': 'I2C_SDA', '44': 'LED_EN', '47': 'GND', '48': '+3V3'}
    B.fp('Package_QFP', 'LQFP-48_7x7mm_P0.5mm', 'U1', 145, 124, 0, 'MCU', mcu)
    for ref, x, y, rot in (('C4', 139, 118, 0), ('C5', 151, 118, 0), ('C6', 151, 130, 0), ('C7', 139, 130, 0)):
        B.fp('Capacitor_SMD', 'C_0603_1608Metric', ref, x, y, rot, '100n', {'1': '+3V3', '2': 'GND'})
    B.fp('Capacitor_SMD', 'C_0603_1608Metric', 'C9', 136, 124, 90, '100n', {'1': 'NRST', '2': 'GND'})
    # --- драйвер светодиодов QFN-32 с открытой площадкой
    drv = {'2': 'I2C_SCL', '3': 'I2C_SDA', '4': 'LED_EN', '25': 'VBUS', '26': 'GND', '27': 'LED_ISET', '33': 'GND'}
    for k in range(8): drv[str(9 + k)] = 'LED_K%d' % k
    B.fp('Package_DFN_QFN', 'QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm', 'U2', 175, 128, 0, 'LED driver', drv)
    B.fp('Capacitor_SMD', 'C_0603_1608Metric', 'C8', 175, 121, 0, '1u', {'1': 'VBUS', '2': 'GND'})
    B.fp('Resistor_SMD', 'R_0603_1608Metric', 'R3', 181, 124, 90, '10k', {'1': 'LED_ISET', '2': 'GND'})
    B.fp('Resistor_SMD', 'R_0603_1608Metric', 'R4', 163, 120, 90, '4.7k', {'1': '+3V3', '2': 'I2C_SCL'})
    B.fp('Resistor_SMD', 'R_0603_1608Metric', 'R5', 166, 120, 90, '4.7k', {'1': '+3V3', '2': 'I2C_SDA'})
    # --- светодиоды: катоды на шину LED_K0..7, аноды на VBUS
    for k in range(8):
        B.fp('LED_SMD', 'LED_0603_1608Metric', 'D%d' % (9 + k), 163 + k * 3.8, 162, 270, 'LED',
             {'1': 'LED_K%d' % k, '2': 'VBUS'})
    # --- матрица кнопок 2×4 с диодами
    n = 0
    for r in range(2):
        for c in range(4):
            n += 1
            x, y = 106 + c * 13, 139 + r * 15
            B.fp('Button_Switch_THT', 'SW_PUSH_6mm', 'SW%d' % n, x, y, 0, 'SW', {'1': 'COL%d' % c, '2': 'Net-(D%d-Pad2)' % n})
            B.fp('Diode_SMD', 'D_SOD-123', 'D%d' % n, x + 3.25, y + 9.0, 0, '1N4148W', {'1': 'ROW%d' % r, '2': 'Net-(D%d-Pad2)' % n})
    # --- разъём: питание и SWD
    B.fp('Connector_PinHeader_2.54mm', 'PinHeader_1x06_P2.54mm_Vertical', 'J2', 192, 108, 0, 'PWR/SWD',
         {'1': '+3V3', '2': 'SWDIO', '3': 'SWCLK', '4': 'NRST', '5': 'GND', '6': 'VBUS'})
    for k, (x, y) in enumerate(((104, 166), (196, 166), (196, 103.5), (104, 128))):
        B.fp('MountingHole', 'MountingHole_3.2mm_M3', 'MH%d' % (k + 1), x, y)
    B.gnd_plane()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    pcbnew.SaveBoard(path, B.b)
    print('записано:', path, '| цепей', len(B.nets), '| футпринтов', len(B.b.GetFootprints()))


if __name__ == '__main__':
    build(sys.argv[1] if len(sys.argv) > 1 else 'example/demo_board.kicad_pcb')
