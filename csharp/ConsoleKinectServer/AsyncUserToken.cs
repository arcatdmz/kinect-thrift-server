using System;
using System.Collections.Generic;
using System.Text;
using System.Net.Sockets;

namespace ConsoleKinectServer
{

    /// <summary>
    /// This class is designed for use as the object to be assigned to the SocketAsyncEventArgs.UserToken property. 
    /// </summary>
    class AsyncUserToken
    {
        Socket m_socket;
        byte[] data;

        public AsyncUserToken() : this(null) { }

        public AsyncUserToken(Socket socket)
        {
            m_socket = socket;
            data = new byte[640*480*4 + 1024 /* 1 + 20*8*5 */];
        }

        public Socket Socket
        {
            get { return m_socket; }
            set { m_socket = value; }
        }

        public byte[] Data
        {
            get { return data; }
            set { data = value; }
        }

        public int Sending;
        public int Sent;
        public int Received;
    }
}
