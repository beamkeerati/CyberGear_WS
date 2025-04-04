#!/usr/bin/python3
#
# cybergear.py
# Libraries for some CyberGear protocols.
# The firmware version of Cybergear must be at least 1.2.1.5.
#
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: (C) 2024 mukyokyo

import math, can
from struct import pack, unpack, iter_unpack

class CyberGear:
    INDEX_run_mode      = (0x7005, 'B')
    INDEX_iq_ref        = (0x7006, 'f')
    INDEX_spd_ref       = (0x700a, 'f')
    INDEX_limit_torque  = (0x700b, 'f')
    INDEX_cur_kp        = (0x7010, 'f')
    INDEX_cur_ki        = (0x7011, 'f')
    INDEX_cur_filt_gain = (0x7014, 'f')
    INDEX_loc_ref       = (0x7016, 'f')
    INDEX_limit_spd     = (0x7017, 'f')
    INDEX_limit_cur     = (0x7018, 'f')
    INDEX_mechPos       = (0x7019, 'f')
    INDEX_iqf           = (0x701a, 'f')
    INDEX_mechVel       = (0x701b, 'f')
    INDEX_VBUS          = (0x701c, 'f')
    INDEX_rotation      = (0x701d, 'h')
    INDEX_loc_kp        = (0x701e, 'f')
    INDEX_spd_kp        = (0x701f, 'f')
    INDEX_spd_ki        = (0x7020, 'f')

    def __init__(self):
        self.__canbus = can.interface.Bus(channel='can0', bustype='socketcan', extended=True)
        self.__myid = 0xfd

    def __send_recieve(self, cmd, id, id_opt, data=(0,0,0,0,0,0,0,0), echo=False):
        if id <= 0x7f:
            try:
                msg = can.Message(
                    arbitration_id=(id & 0xffff) | (((id_opt | (self.__myid & 0xffff)) & 0xffffffff) << 8) | ((cmd & 0x1f) << 24),
                    data=data
                )
                if echo:
                    print(f'TX: id=${msg.arbitration_id:08x} data=', ':'.join(format(x, "02x") for x in msg.data))
                self.__canbus.send(msg)
                r = self.__canbus.recv(timeout=0.05)
                if r is not None:
                    if echo:
                        print(f'RX: id=${r.arbitration_id:08X} data=', ':'.join(format(x, "02x") for x in r.data))
                    return r
            except:
                pass

    # get unique identifier
    def type0(self, id, echo=False):
        r = self.__send_recieve(0, id, 0, echo=echo)
        if r is not None:
            return list(*iter_unpack("<Q", bytes(r.data)))[0]

    # control mode
    def type1(self, id, torque, angle, speed, Kp, Kd, echo=False):
        r = self.__send_recieve(1, id, torque, bytes(pack(">HHHH", angle, speed, Kp, Kd)), echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r)))

    # feedback
    def type2(self, id, echo=False):
        r = self.__send_recieve(2, id, 0, echo=echo)
        if r is not None:
            return r.arbitration_id, tuple(*iter_unpack(">HHHH", bytes(r.data)))

    # enable
    def type3(self, id, echo=False):
        r = self.__send_recieve(3, id, 0, echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r.data)))

    # shutdown
    def type4(self, id, fault=False, echo=False):
        r = self.__send_recieve(4, id, 0, (1 if fault else 0, 0, 0, 0, 0, 0, 0, 0), echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r.data)))

    # set zero position
    def type6(self, id, echo=False):
        r = self.__send_recieve(6, id, 0, (1, 0, 0, 0, 0, 0, 0, 0), echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r.data)))

    # change id
    def type7(self, id, newid, echo=False):
        if newid <= 0x7f:
            return self.__send_recieve(7, id, newid << 8, echo=echo)

    # read config
    def type9(self, id, index, echo=False):
        if id <= 0x7f:
            s = bytes()
            try:
                cmd = 9
                id_opt = 0
                msg = can.Message(
                    arbitration_id=(id & 0xffff) | (((id_opt | (self.__myid & 0xffff)) & 0xffffffff) << 8) | ((cmd & 0x1f) << 24),
                    data=bytes(pack("<Hxxxxxx", index))
                )
                if echo:
                    print(f'TX: id=${msg.arbitration_id:08x} data=', ':'.join(format(x, "02x") for x in msg.data))
                self.__canbus.send(msg)
                lcnt = 3
                cnt = 0
                while lcnt >= cnt:
                    r = self.__canbus.recv(timeout=0.05)
                    if r is not None:
                        if echo:
                            print(f'RX: id=${r.arbitration_id:08X} data=', ':'.join(format(x, "02x") for x in r.data))
                        d = tuple(*iter_unpack("<HBBBBBB", bytes(r.data)))
                        s += bytes(d[3:7])
                        match d[1]:
                            case 0x00:  # uint8
                                lcnt = 0
                            case 0x02:  # uint16
                                lcnt = 0
                            case 0x03:  # int16
                                lcnt = 0
                            case 0x04:  # uint32
                                lcnt = 0
                            case 0x05:  # int32
                                lcnt = 0
                            case 0x06:  # float
                                lcnt = 0
                            case 0x0a:  # str
                                lcnt = 3
                    else:
                        break
                    cnt += 1
                match d[1]:
                    case 0x00:  # uint8
                        return tuple(*iter_unpack("<Bxxx", s))[0]
                    case 0x02:  # uint16
                        return tuple(*iter_unpack("<Hxx", s))[0]
                    case 0x03:  # int16
                        return tuple(*iter_unpack("<hxx", s))[0]
                    case 0x04:  # uint32
                        return tuple(*iter_unpack("<I", s))[0]
                    case 0x05:  # int32
                        return tuple(*iter_unpack("<i", s))[0]
                    case 0x06:  # float
                        return tuple(*iter_unpack("<f", s))[0]
                    case 0x0a:  # str
                        ss = s.decode("utf-8").split("\0")
                        return ss[0]
            except:
                pass

    # read param
    def type17(self, id, index, width="B", echo=False):
        r = self.__send_recieve(17, id, 0, bytes(pack("<Hxxxxxx", index)), echo=echo)
        if r is not None:
            match width:
                case "B":
                    return tuple(*iter_unpack("<HxxBxxx", bytes(r.data)))[1]
                case "h":
                    return tuple(*iter_unpack("<Hxxhxx", bytes(r.data)))[1]
                case "H":
                    return tuple(*iter_unpack("<HxxHxx", bytes(r.data)))[1]
                case "l":
                    return tuple(*iter_unpack("<Hxxl", bytes(r.data)))[1]
                case "L":
                    return tuple(*iter_unpack("<HxxL", bytes(r.data)))[1]
                case "f":
                    return tuple(*iter_unpack("<Hxxf", bytes(r.data)))[1]

    # write param
    def type18(self, id, index, data, width="B", echo=False):
        r = None
        match width:
            case "B":
                r = self.__send_recieve(18, id, 0, bytes(pack("<HxxBxxx", index, data)), echo=echo)
            case "h":
                r = self.__send_recieve(18, id, 0, bytes(pack("<Hxxhxx", index, data)), echo=echo)
            case "H":
                r = self.__send_recieve(18, id, 0, bytes(pack("<HxxHxx", index, data)), echo=echo)
            case "l":
                r = self.__send_recieve(18, id, 0, bytes(pack("<Hxxl", index, data)), echo=echo)
            case "L":
                r = self.__send_recieve(18, id, 0, bytes(pack("<HxxL", index, data)), echo=echo)
            case "f":
                r = self.__send_recieve(18, id, 0, bytes(pack("<Hxxf", index, data)), echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r.data)))

    # read raw config
    def type19(self, id, echo=False):
        if id <= 0x7f:
            s = bytes()
            try:
                cmd = 19
                id_opt = 0
                msg = can.Message(
                    arbitration_id=(id & 0xffff) | (((id_opt | (self.__myid & 0xffff)) & 0xffffffff) << 8) | ((cmd & 0x1f) << 24),
                    data=(0xC4, 0x7F, 0x31, 0x31, 0x30, 0x33, 0x31, 0x04)
                )
                if echo:
                    print(f'TX: id=${msg.arbitration_id:08x} data=', ':'.join(format(x, "02x") for x in msg.data))
                self.__canbus.send(msg)
                while True:
                    r = self.__canbus.recv(timeout=0.05)
                    if r is not None:
                        if echo:
                            print(f'RX: id=${r.arbitration_id:08X} data=', ':'.join(format(x, "02x") for x in r.data))
                        d = tuple(*iter_unpack("<HBBBBBB", bytes(r.data)))
                        if r.arbitration_id & 0xf0000 != 0x90000:
                            s += bytes(d[1:7])
                        else:
                            break
                    else:
                        break
            except:
                pass
            return s

    def shutdown(self):
        self.__canbus.shutdown()


if __name__ == "__main__":
    import sys, math, time

    cg = CyberGear()

    def float_to_uint(x: float, x_min: float, x_max: float):
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int((x - x_min) * 65535 / (x_max - x_min))

    def uint_to_float(x: int, x_min: float, x_max: float):
        return float(x / 65535 * (x_max - x_min) + x_min)

    def get_unique_identifier(id, echo=False):
        return cg.type0(id, echo=echo)

    def get_feedback(id, echo=False):
        r = cg.type2(id, echo=echo)
        if r is not None:
            return (
                (r[0] >> 16) & 0xff,
                uint_to_float(r[1][0], -4 * math.pi, 4 * math.pi),
                uint_to_float(r[1][1], -30, 30),
                uint_to_float(r[1][2], -12, 12),
                uint_to_float(r[1][3], 0, 6553.5)
            )
        return None

    def start_motor(id, echo=False):
        cg.type3(id, echo=echo)

    def stop_motor(id, fault=False, echo=False):
        cg.type4(id, fault, echo=echo)

    def set_newid(id, newid, echo=False):
        cg.type7(id, newid, echo=echo)

    def set_runmode(id, m, echo=False):
        stop_motor(id)
        cg.type18(id, CyberGear.INDEX_run_mode[0], m, width=CyberGear.INDEX_run_mode[1], echo=echo)

    def set_zero_position(id, echo=False):
        cg.type6(id, echo=echo)

    def set_iq_ref(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_iq_ref[0], v, width=CyberGear.INDEX_iq_ref[1], echo=echo)

    def set_spd_ref(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_spd_ref[0], v, width=CyberGear.INDEX_spd_ref[1], echo=echo)

    def set_limit_torque(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_limit_torque[0], v, width=CyberGear.INDEX_limit_torque[1], echo=echo)

    def set_cur_kp(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_cur_kp[0], v, width=CyberGear.INDEX_cur_kp[1], echo=echo)

    def set_cur_ki(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_cur_ki[0], v, width=CyberGear.INDEX_cur_ki[1], echo=echo)

    def set_cur_filt_gain(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_cur_filt_gain[0], v, width=CyberGear.INDEX_cur_filt_gain[1], echo=echo)

    def set_loc_ref(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_loc_ref[0], v, width=CyberGear.INDEX_loc_ref[1], echo=echo)

    def set_limit_spd(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_limit_spd[0], v, width=CyberGear.INDEX_limit_spd[1], echo=echo)

    def set_limit_cur(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_limit_cur[0], v, width=CyberGear.INDEX_limit_cur[1], echo=echo)

    def set_rotation(id, v, echo=False):
        cg.type18(id, CyberGear.INDEX_rotation[0], v, width=CyberGear.INDEX_rotation[1], echo=echo)

    def get_rotation(id, echo=False):
        return cg.type17(id, CyberGear.INDEX_rotation[0], width=CyberGear.INDEX_rotation[1], echo=echo)

    def dump1(id, echo=False):
        result = ()
        for i in (0x0, 0x1, 0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007,
                  0x2000, 0x2001, 0x2002, 0x2003, 0x2004, 0x2005, 0x2006, 0x2007, 0x2008, 0x2009,
                  0x200a, 0x200b, 0x200c, 0x200d, 0x200e, 0x200f, 0x2010, 0x2011, 0x2012, 0x2013,
                  0x2014, 0x2015, 0x2016, 0x2017, 0x2018, 0x2019, 0x3000, 0x3001, 0x3002, 0x3003,
                  0x3004, 0x3005, 0x3006, 0x3007, 0x3008, 0x3009, 0x300a, 0x300b, 0x300c, 0x300d,
                  0x300e, 0x300f, 0x3010, 0x3011, 0x3012, 0x3013, 0x3014, 0x3015, 0x3016, 0x3017,
                  0x3018, 0x3019, 0x301a, 0x301b, 0x301c, 0x301d, 0x301e, 0x301f, 0x3020, 0x3021,
                  0x3022, 0x3023, 0x3024, 0x3025, 0x3026, 0x3027, 0x3028, 0x3029, 0x302a, 0x302b,
                  0x302c, 0x302d, 0x302e, 0x302f):
            d = cg.type9(id, i)
            if echo:
                print(f'${i:04x} {d}')
            result += ((i, d),)
        return result

    def dump2(id, echo=False):
        result = ()
        for i in (CyberGear.INDEX_run_mode, CyberGear.INDEX_iq_ref, CyberGear.INDEX_spd_ref,
                  CyberGear.INDEX_limit_torque, CyberGear.INDEX_cur_kp, CyberGear.INDEX_cur_ki,
                  CyberGear.INDEX_cur_filt_gain, CyberGear.INDEX_loc_ref, CyberGear.INDEX_limit_spd,
                  CyberGear.INDEX_limit_cur, CyberGear.INDEX_mechPos, CyberGear.INDEX_iqf,
                  CyberGear.INDEX_mechVel, CyberGear.INDEX_VBUS, CyberGear.INDEX_rotation,
                  CyberGear.INDEX_loc_kp, CyberGear.INDEX_spd_kp, CyberGear.INDEX_spd_ki):
            r = cg.type17(id, i[0], width=i[1])
            if r is not None:
                result += ((i[0], r),)
                if echo:
                    match i[1]:
                        case "B":
                            print(f'${i[0]:04x} ${r:02x}')
                        case "f":
                            print(f'${i[0]:04x} {r:.3f}')
                        case "h":
                            print(f'${i[0]:04x} {r}')
        return result

    def dump3(id, echo=False):
        result = ()
        # Only query the three specific indices
        for i in (0x3016, 0x3017, 0x302d, 0x301e):
            d = cg.type9(id, i)
            if echo:
                print(f'${i:04x} {d}')
            result += ((i, d),)
        return result

    id1 = 127
    print("start")

    dump1(id1, echo=True)
    dump2(id1, echo=True)
    set_zero_position(id1)
    # input()
    # print(hex(get_unique_identifier(id1)))
    stop_motor(id1, True)

    # pos: -4pi ~ +4pi = 0 ~ 65535
    # spd: -30 rad/s ~ +30 rad/s = 0 ~ 65535
    # trq: -12 Nm ~ +12 Nm = 0 ~ 65535
    try:
        print("\ncurrent mode")
        set_runmode(id1, 3)
        start_motor(id1)
        with open("output.txt", "w") as output_file:
            for c in tuple(range(0, 23, 1)) + tuple(range(23, -23, -1)) + tuple(range(-23, 0, 1)):
                set_iq_ref(id1, c)
                t = time.time() + 0.1
                while t > time.time():
                    d1 = dump3(id1)
                    # Initialize variables to store the values
                    pos = None
                    vel = None
                    torq = None
                    iq = None
                    if d1 is not None:
                        for v1 in d1:
                            # Collect values based on parameter index
                            if v1[0] == 0x3016:
                                pos = v1[1]
                            elif v1[0] == 0x3017:
                                vel = v1[1]
                            elif v1[0] == 0x302d:
                                torq = v1[1]
                            elif v1[0] == 0x301e:
                                iq = v1[1]
                        # Check if all values have been read before printing
                        if pos is not None and vel is not None and torq is not None:
                            text = f"Pos: {pos:10.8f} Vel: {vel:10.8f} Torq: {torq:10.8f} Iq: {iq:10.8f}"
                            print(text)
                            output_file.write(text + "\n")
    except KeyboardInterrupt:
        pass

    stop_motor(id1)
    cg.shutdown()
    print()
