
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.Windows.Threading;
using ExtendedSerialPort;

namespace WpfApp1
{
    static class Message
    {
        public static byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checkSum;

            checkSum = 0xFE;
            checkSum ^= (byte) msgFunction;
            checkSum ^= (byte) (msgFunction >> 8);
            checkSum ^= (byte) msgPayloadLength;
            checkSum ^= (byte) (msgPayloadLength >> 8);
            for (int i = 0; i < msgPayloadLength; i++)
                checkSum ^= (byte)msgPayload[i];

            return checkSum;
        }

        public static byte[] UartEncodeMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte[] message = new byte[msgPayloadLength + 6];
            int index = 0;
            message[index++] = 0xFE;
            message[index++] = (byte)(msgFunction >> 8);
            message[index++] = (byte)msgFunction;
            message[index++] = (byte)(msgPayloadLength>>8);
            message[index++] = (byte)msgPayloadLength;
            for (int i = 0; i < msgPayloadLength; i++)
                message[index++] = msgPayload[i];
            message[index++] = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            return message;
        }
    }

}
