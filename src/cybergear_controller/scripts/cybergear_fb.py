#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math, can
from struct import pack, unpack, iter_unpack
import sys, time
from std_msgs.msg import Int16,Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
        self.__canbus = can.interface.Bus(channel='can0', interface='socketcan', extended=True)
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
            except Exception as e:
                if echo:
                    print("Error in __send_recieve:", e)

    # get unique identifier
    def type0(self, id, echo=False):
        r = self.__send_recieve(0, id, 0, echo=echo)
        if r is not None:
            return list(*iter_unpack("<Q", bytes(r.data)))[0]

    # control mode
    def type1(self, id, torque, angle, speed, Kp, Kd, echo=False):
        r = self.__send_recieve(1, id, torque, bytes(pack(">HHHH", angle, speed, Kp, Kd)), echo=echo)
        if r is not None:
            return list(*iter_unpack(">HHHH", bytes(r.data)))

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
            except Exception as e:
                if echo:
                    print("Error in type9:", e)
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
            except Exception as e:
                if echo:
                    print("Error in type19:", e)
                pass
            return s

    def shutdown(self):
        self.__canbus.shutdown()


class CBG_CMD_Node(Node):

    def __init__(self):
        super().__init__('cybergear_cmd_node')
        self.cg = CyberGear()  # store CyberGear instance as attribute

        self.ids = [1, 2, 3, 4, 5, 6]
        # for id in self.ids:
        #     self.set_runmode(id, 2)
        #     self.set_limit_cur(id, 23.0)
        #     self.start_motor(id)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_states', qos_profile)
        self.timer_period = 1.0/1500.0  # seconds (10 ms)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.j_states = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.get_logger().info("CyberGear_fb_node has been executed")

    def timer_callback(self):

        for id in self.ids:
            d1 = self.dump3(id)
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
                         self.j_states[id-1] = pos
                     elif v1[0] == 0x3017:
                         vel = v1[1]
                     elif v1[0] == 0x302d:
                         torq = v1[1]
                     elif v1[0] == 0x301e:
                         iq = v1[1]
                # Check if all values have been read before printing
                #if pos is not None and vel is not None and torq is not None:
                #     text = f"Pos: {pos:10.8f} Vel: {vel:10.8f} Torq: {torq:10.8f} Iq: {iq:10.8f}"
                #     print(text)
        # Create a Float32MultiArray message with 6 identical elements
        try:
            msg = Float32MultiArray()
            msg.data = self.j_states
            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published: {msg.data}')
        except:
            pass


    def float_to_uint(self, x: float, x_min: float, x_max: float):
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int((x - x_min) * 65535 / (x_max - x_min))

    def uint_to_float(self, x: int, x_min: float, x_max: float):
        return float(x / 65535 * (x_max - x_min) + x_min)

    def get_unique_identifier(self, id, echo=False):
        return self.cg.type0(id, echo=echo)

    def get_feedback(self, id, echo=False):
        r = self.cg.type2(id, echo=echo)
        if r is not None:
            return (
                (r[0] >> 16) & 0xff,
                self.uint_to_float(r[1][0], -4 * math.pi, 4 * math.pi),
                self.uint_to_float(r[1][1], -30, 30),
                self.uint_to_float(r[1][2], -12, 12),
                self.uint_to_float(r[1][3], 0, 6553.5)
            )
        return None

    def start_motor(self, id, echo=False):
        self.cg.type3(id, echo=echo)

    def stop_motor(self, id, fault=False, echo=False):
        self.cg.type4(id, fault, echo=echo)

    def set_newid(self, id, newid, echo=False):
        self.cg.type7(id, newid, echo=echo)

    def set_runmode(self, id, m, echo=False):
        self.stop_motor(id)
        self.cg.type18(id, CyberGear.INDEX_run_mode[0], m,
                       width=CyberGear.INDEX_run_mode[1], echo=echo)

    def set_zero_position(self, id, echo=False):
        self.cg.type6(id, echo=echo)

    def set_iq_ref(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_iq_ref[0], v,
                       width=CyberGear.INDEX_iq_ref[1], echo=echo)

    def set_spd_ref(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_spd_ref[0], v,
                       width=CyberGear.INDEX_spd_ref[1], echo=echo)

    def set_limit_torque(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_limit_torque[0], v,
                       width=CyberGear.INDEX_limit_torque[1], echo=echo)

    def set_cur_kp(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_cur_kp[0], v,
                       width=CyberGear.INDEX_cur_kp[1], echo=echo)

    def set_cur_ki(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_cur_ki[0], v,
                       width=CyberGear.INDEX_cur_ki[1], echo=echo)

    def set_cur_filt_gain(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_cur_filt_gain[0], v,
                       width=CyberGear.INDEX_cur_filt_gain[1], echo=echo)

    def set_loc_ref(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_loc_ref[0], v,
                       width=CyberGear.INDEX_loc_ref[1], echo=echo)

    def set_limit_spd(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_limit_spd[0], v,
                       width=CyberGear.INDEX_limit_spd[1], echo=echo)

    def set_limit_cur(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_limit_cur[0], v,
                       width=CyberGear.INDEX_limit_cur[1], echo=echo)

    def set_rotation(self, id, v, echo=False):
        self.cg.type18(id, CyberGear.INDEX_rotation[0], v,
                       width=CyberGear.INDEX_rotation[1], echo=echo)

    def get_rotation(self, id, echo=False):
        return self.cg.type17(id, CyberGear.INDEX_rotation[0],
                              width=CyberGear.INDEX_rotation[1], echo=echo)

    def dump1(self, id, echo=False):
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
            d = self.cg.type9(id, i)
            if echo:
                print(f'${i:04x} {d}')
            result += ((i, d),)
        return result

    def dump2(self, id, echo=False):
        result = ()
        for i in (CyberGear.INDEX_run_mode, CyberGear.INDEX_iq_ref, CyberGear.INDEX_spd_ref,
                  CyberGear.INDEX_limit_torque, CyberGear.INDEX_cur_kp, CyberGear.INDEX_cur_ki,
                  CyberGear.INDEX_cur_filt_gain, CyberGear.INDEX_loc_ref, CyberGear.INDEX_limit_spd,
                  CyberGear.INDEX_limit_cur, CyberGear.INDEX_mechPos, CyberGear.INDEX_iqf,
                  CyberGear.INDEX_mechVel, CyberGear.INDEX_VBUS, CyberGear.INDEX_rotation,
                  CyberGear.INDEX_loc_kp, CyberGear.INDEX_spd_kp, CyberGear.INDEX_spd_ki):
            r = self.cg.type17(id, i[0], width=i[1])
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

    def dump3(self, id, echo=False):
        result = ()
        # Only query the three specific indices
        for i in (0x3016, 0x3017, 0x302d, 0x301e):
            d = self.cg.type9(id, i)
            if echo:
                print(f'${i:04x} {d}')
            result += ((i, d),)
        return result




def main(args=None):
    rclpy.init(args=args)
    node = CBG_CMD_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
