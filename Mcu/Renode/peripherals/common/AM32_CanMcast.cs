//
// Bridges the emulated CAN controller to the SITL's multicast CAN bus,
// so the emulated ESC talks DroneCAN with anything that speaks the
// ArduPilot mcast scheme - dronecan_gui_tool on "mcast:N", the SITL
// GUI's DroneCAN panel, sitl_can_test.py.
//
// Wire format, byte-compatible with Src/DroneCAN/sys_can_SITL.c and
// libcanard's drivers/mcast: UDP datagrams to 239.65.82.<bus>:57732,
//
//   u16 magic 0x2934, u16 crc, u16 flags, u32 message_id, u8 data[dlc]
//
// all little-endian, dlc implied by the datagram length, crc16-CCITT
// over flags..end. message_id is the libcanard frame id verbatim:
// bit 31 is the extended-frame flag (every DroneCAN frame), bit 30 RTR.
//
// This is an ICAN: it joins the same CANHub as the CAN controller and
// converts frames both ways. It is registered on the sysbus like the
// other made-up peripherals so the monitor can address it; the
// registers are a debug window, not part of any real device.
//
// Multicast loops back to the sender's own host, so the receive path
// drops datagrams whose source is our own transmit socket - the same
// check sys_can_SITL.c makes - or every frame the ESC sent would come
// straight back at it.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Antmicro.Renode.Peripherals.CAN
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_CanMcast : IDoubleWordPeripheral, IKnownSize, ICAN,
                                 IDisposable
    {
        public AM32_CanMcast(IMachine machine)
        {
            this.machine = machine;
            bus = -1;
        }

        public event Action<CANMessageFrame> FrameSent;

        public long Size => 0x100;

        // A firmware self-reset must not take the bus link down: the
        // node comes back and keeps talking, as on a real bus.
        public void Reset()
        {
        }

        // machine teardown, unlike firmware reset, must not leave a
        // ghost thread receiving alongside a replacement machine
        public void Dispose()
        {
            lock(lifecycle)
            {
                Close();
            }
        }

        // the mcast bus number: 239.65.82.<Bus>. Setting it opens the
        // sockets; negative closes them. Left closed by default so a
        // run that is not using CAN cannot collide with a real SITL on
        // the same machine. Serialized: reconfiguring while the old
        // receive thread is still draining must not race it.
        // join and transmit on the loopback interface only, keeping the
        // bus private to this machine
        public bool LoopbackOnly { get; set; }

        public int Bus
        {
            get { return bus; }
            set
            {
                if(value > 9)
                {
                    // the SITL's scheme is 239.65.82.<bus>, one octet
                    // shared with nothing else only for 0..9
                    throw new RecoverableException(string.Format(
                        "mcast bus {0} is out of range, expected 0..9", value));
                }
                lock(lifecycle)
                {
                    if(value == bus)
                    {
                        return;
                    }
                    Close();
                    if(value < 0)
                    {
                        return;
                    }
                    Open(value);
                }
            }
        }

        // debug window: 0x00 bus, 0x04 frames ESC->host, 0x08 host->ESC
        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return (uint)bus;
            case 0x04: return framesToHost;
            case 0x08: return framesFromHost;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x00)
            {
                Bus = (int)value;
            }
        }

        public uint FramesToHost => framesToHost;
        public uint FramesFromHost => framesFromHost;

        // a frame from the hub: the ESC transmitted it
        public void OnFrameReceived(CANMessageFrame message)
        {
            var tx = txSocket;
            if(tx == null || message.Data.Length > 8)
            {
                return;
            }
            SendToHost(tx, message);
        }

        private void SendToHost(Socket tx, CANMessageFrame message)
        {
            var id = message.ExtendedFormat
                ? ((message.Id & 0x1FFFFFFFu) | EffFlag)
                : (message.Id & 0x7FFu);
            if(message.RemoteFrame)
            {
                id |= RtrFlag;
            }
            var pkt = new byte[HeaderLen + message.Data.Length];
            pkt[0] = (byte)(Magic & 0xFF);
            pkt[1] = (byte)(Magic >> 8);
            // flags: no CANFD
            pkt[4] = 0;
            pkt[5] = 0;
            pkt[6] = (byte)id;
            pkt[7] = (byte)(id >> 8);
            pkt[8] = (byte)(id >> 16);
            pkt[9] = (byte)(id >> 24);
            Array.Copy(message.Data, 0, pkt, HeaderLen, message.Data.Length);
            var crc = Crc16(pkt, 4, pkt.Length - 4);
            pkt[2] = (byte)crc;
            pkt[3] = (byte)(crc >> 8);
            try
            {
                tx.Send(pkt);
                framesToHost++;
            }
            catch(SocketException e)
            {
                this.Log(LogLevel.Warning, "mcast send failed: {0}", e.Message);
            }
            catch(ObjectDisposedException)
            {
                // a concurrent Bus change closed the socket under us
            }
        }

        // Callers hold `lifecycle`. The sockets are created as locals and
        // only published once fully set up, and the receive thread gets
        // them as arguments rather than reading the mutable fields - a
        // reconfigure must never hand the thread a null or someone
        // else's socket.
        private void Open(int busNumber)
        {
            var group = IPAddress.Parse(string.Format("239.65.82.{0}", busNumber));
            Socket tx = null;
            Socket rx = null;
            try
            {
                // separate transmit socket, so its ephemeral local port
                // identifies our own datagrams on the receive side
                var lo = IPAddress.Loopback;
                tx = new Socket(AddressFamily.InterNetwork,
                                SocketType.Dgram, ProtocolType.Udp);
                tx.SetSocketOption(SocketOptionLevel.IP,
                                   SocketOptionName.MulticastTimeToLive, 1);
                if(LoopbackOnly)
                {
                    // multicast out through lo, so the frames never leave
                    // the machine and LAN traffic on the same group (a
                    // busy bench network reaches every bus number) never
                    // reaches the emulated ESC
                    tx.SetSocketOption(SocketOptionLevel.IP,
                                       SocketOptionName.MulticastInterface,
                                       lo.GetAddressBytes());
                }
                tx.Connect(new IPEndPoint(group, Port));

                rx = new Socket(AddressFamily.InterNetwork,
                                SocketType.Dgram, ProtocolType.Udp);
                rx.SetSocketOption(SocketOptionLevel.Socket,
                                   SocketOptionName.ReuseAddress, true);
                // bind the group address itself, as the SITL does, so
                // another bus number on the same port stays invisible
                rx.Bind(new IPEndPoint(group, Port));
                rx.SetSocketOption(SocketOptionLevel.IP,
                                   SocketOptionName.AddMembership,
                                   LoopbackOnly ? new MulticastOption(group, lo)
                                                : new MulticastOption(group));
            }
            catch(SocketException e)
            {
                if(tx != null)
                {
                    tx.Close();
                }
                if(rx != null)
                {
                    rx.Close();
                }
                throw new RecoverableException(string.Format(
                    "cannot open mcast bus {0}: {1}", busNumber, e.Message));
            }
            txSocket = tx;
            rxSocket = rx;
            bus = busNumber;
            rxThread = new Thread(() => ReceiveLoop(rx, tx))
            {
                IsBackground = true,
                Name = "am32 can mcast " + busNumber,
            };
            rxThread.Start();
            this.Log(LogLevel.Info, "CAN bridged to mcast bus {0} ({1}:{2})",
                     busNumber, group, Port);
        }

        // Callers hold `lifecycle`. Joining the old thread before the
        // caller opens a replacement means a stale datagram can never be
        // delivered as a frame on the new bus.
        private void Close()
        {
            bus = -1;
            var tx = txSocket;
            var rx = rxSocket;
            var thread = rxThread;
            txSocket = null;
            rxSocket = null;
            rxThread = null;
            // closing the rx socket is what stops the receive thread
            if(rx != null)
            {
                rx.Close();
            }
            if(tx != null)
            {
                tx.Close();
            }
            if(thread != null && !thread.Join(2000))
            {
                this.Log(LogLevel.Warning,
                         "mcast receive thread did not stop in time");
            }
        }

        // Socket thread. An unhandled exception on a background thread
        // takes the whole emulator down with it, so everything is caught;
        // an ObjectDisposedException is the normal Close() path.
        private void ReceiveLoop(Socket rx, Socket tx)
        {
            var own = (IPEndPoint)tx.LocalEndPoint;
            var buf = new byte[64];
            EndPoint from = new IPEndPoint(IPAddress.Any, 0);
            try
            {
                while(true)
                {
                    int n;
                    try
                    {
                        from = new IPEndPoint(IPAddress.Any, 0);
                        n = rx.ReceiveFrom(buf, ref from);
                    }
                    catch(SocketException)
                    {
                        if(rxSocket != rx)
                        {
                            return; // replaced or closed; this thread is done
                        }
                        continue;
                    }
                    if(rxSocket != rx)
                    {
                        return;
                    }
                    var src = (IPEndPoint)from;
                    // our own transmissions loop back; drop them
                    if(src.Port == own.Port
                       && (src.Address.Equals(own.Address)
                           || IPAddress.IsLoopback(src.Address)
                           || IsLocalAddress(src.Address)))
                    {
                        continue;
                    }
                    // Joining the group on lo does not stop delivery of
                    // frames another process's membership pulled in from
                    // the LAN, so a private bus needs a source filter
                    // too: only this machine's own senders count.
                    if(LoopbackOnly && !IPAddress.IsLoopback(src.Address)
                       && !IsLocalAddress(src.Address))
                    {
                        continue;
                    }
                    var frame = Decode(buf, n);
                    if(frame == null)
                    {
                        continue;
                    }
                    framesFromHost++;
                    var handler = FrameSent;
                    if(handler != null)
                    {
                        // delivered on this thread, as Renode's own
                        // SocketCANBridge does
                        handler(frame);
                    }
                }
            }
            catch(ObjectDisposedException)
            {
                // Close() ran; the thread's work is done
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "mcast receive thread stopped: {0}", e);
            }
        }

        private CANMessageFrame Decode(byte[] buf, int n)
        {
            if(n < HeaderLen || n > HeaderLen + 8)
            {
                return null;
            }
            var magic = (ushort)(buf[0] | (buf[1] << 8));
            if(magic != Magic)
            {
                return null;
            }
            var crc = (ushort)(buf[2] | (buf[3] << 8));
            if(crc != Crc16(buf, 4, n - 4))
            {
                return null;
            }
            var flags = (ushort)(buf[4] | (buf[5] << 8));
            if((flags & FlagCanFd) != 0)
            {
                return null; // classic CAN cannot receive FD frames
            }
            var id = (uint)(buf[6] | (buf[7] << 8) | (buf[8] << 16)
                            | ((uint)buf[9] << 24));
            if((id & ErrFlag) != 0)
            {
                return null; // libcanard error frames are not deliverable
            }
            var extended = (id & EffFlag) != 0;
            if(!extended && (id & ~RtrFlag) > 0x7FF)
            {
                // a standard frame with upper identifier bits is
                // malformed; rejecting beats silently aliasing it
                return null;
            }
            var data = new byte[n - HeaderLen];
            Array.Copy(buf, HeaderLen, data, 0, data.Length);
            return new CANMessageFrame(
                id & (extended ? 0x1FFFFFFFu : 0x7FFu), data,
                extendedFormat: extended,
                remoteFrame: (id & RtrFlag) != 0);
        }

        // is this one of our own interface addresses? A datagram from
        // another process on this host must NOT be dropped, so this is
        // only consulted together with the matching source port.
        private static bool IsLocalAddress(IPAddress addr)
        {
            try
            {
                foreach(var a in Dns.GetHostAddresses(Dns.GetHostName()))
                {
                    if(a.Equals(addr))
                    {
                        return true;
                    }
                }
            }
            catch(Exception)
            {
                // name resolution failure: fall through to "not local"
            }
            return false;
        }

        // CRC16-CCITT, init 0xFFFF, poly 0x1021, MSB first - the same
        // crc16_CCITT sys_can_SITL.c uses
        private static ushort Crc16(byte[] buf, int offset, int len)
        {
            ushort crc = 0xFFFF;
            for(var i = 0; i < len; i++)
            {
                crc ^= (ushort)(buf[offset + i] << 8);
                for(var b = 0; b < 8; b++)
                {
                    crc = (crc & 0x8000) != 0
                        ? (ushort)((crc << 1) ^ 0x1021)
                        : (ushort)(crc << 1);
                }
            }
            return crc;
        }

        private const ushort Magic = 0x2934;
        private const ushort FlagCanFd = 0x0001;
        private const uint EffFlag = 0x80000000u;
        private const uint RtrFlag = 0x40000000u;
        private const uint ErrFlag = 0x20000000u;
        private const int Port = 57732;
        private const int HeaderLen = 10;

        private readonly IMachine machine;
        // serializes Open/Close against Bus reconfiguration
        private readonly object lifecycle = new object();

        private volatile Socket txSocket;
        private volatile Socket rxSocket;
        private Thread rxThread;
        private int bus;
        private uint framesToHost;
        private uint framesFromHost;
    }
}
