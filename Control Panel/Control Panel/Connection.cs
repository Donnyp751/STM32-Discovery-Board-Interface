using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.IO.Ports;
using System.Threading;
using System.Timers;
using System.Windows.Media.Animation;
using Timer = System.Timers.Timer;

namespace Control_Panel
{
    class MessageEventArgs : EventArgs
    {
        public string Message;
        public MessageEventArgs(string message)
        {
            Message = message;
        }
    }
    class Connection : INotifyPropertyChanged
    {
        private Timer updateTimer;
        private enum connectionStatusEnum
        {
            Disconnected,
            Connected,
            Error
        }
        connectionStatusEnum connectionStatus;

        private string _connectionStatus;
        private ObservableCollection<string> comPorts;
        private ObservableCollection<string> consoleOutput;
        public event PropertyChangedEventHandler PropertyChanged;
        private SerialPort serialPort;
        public delegate void NewSerialMessage(object sender, MessageEventArgs args);
        string output = "";
        private bool ConsoleConnectionLock = false;

        public event NewSerialMessage OnSerialMessageReceived;
        

        public bool IsOpen;
        public string ConnectionStatus
        {
            get => _connectionStatus;
            set
            {
                if (!string.IsNullOrEmpty(value))
                {
                    _connectionStatus = value;
                    NotifyPropertyChanged();
                    //PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("ConnectionStatus"));
                }
                    
            }
        }
        private connectionStatusEnum _ConnectionStatus
        {
            get => connectionStatus; 
            set
            {
                connectionStatus = value;
                switch (value)
                {
                    case connectionStatusEnum.Connected:
                        ConnectionStatus = "Connected";
                        IsOpen = true;
                        break;

                    case connectionStatusEnum.Disconnected:
                        ConnectionStatus = "Disconnected";
                        IsOpen = false;
                        break;

                    case connectionStatusEnum.Error:
                        ConnectionStatus = "Error";
                        IsOpen = false;
                        break;
                }
            }
        }
        public ObservableCollection<string> ComPorts
        {
            get => comPorts;
            set
            {
                if (value != null)
                {
                    comPorts = value;
                    NotifyPropertyChanged();
                    //PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("ComPorts"));
                }
            }

        }


        public string ConsoleOutput
        {
            get
            {
                while (ConsoleConnectionLock)
                {
                    Thread.Sleep(10);
                    //spin till unlocked
                }

                ConsoleConnectionLock = true;
                foreach (var line in consoleOutput)
                {
                    output += line.Replace('\t', '\n');

                }
                consoleOutput.Clear();
                ConsoleConnectionLock = false;
                return output;
            }
            set
            {
                if (value != null)
                {
                    while (ConsoleConnectionLock)
                    {
                        Thread.Sleep(10);
                        //spin till unlocked
                    }
                    ConsoleConnectionLock = true;

                    consoleOutput.Add(value);
                    ConsoleConnectionLock = false;

                    NotifyPropertyChanged();
                }
            }
        }
        private void NotifyPropertyChanged([CallerMemberName] String propertyName = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
        public Connection()
        {
            updateTimer = new Timer();
            consoleOutput = new ObservableCollection<string>();
            updateTimer.Interval = 50;
            updateTimer.AutoReset = true;
            updateTimer.Elapsed += TimedUpdate;
            updateTimer.Start();

            _ConnectionStatus = connectionStatusEnum.Disconnected;
            updateComPorts();
        }

        public string GetLatestMessage()
        {
            return consoleOutput.Last();
        }
        private void TimedUpdate(object sender, ElapsedEventArgs e)
        {
            if (serialPort == null) return;


            IsOpen = serialPort.IsOpen;
            if (serialPort.IsOpen)
            {
                _ConnectionStatus = connectionStatusEnum.Connected;

            }
            else
            {
                connectionStatus = connectionStatusEnum.Disconnected;
            }
        }

        //Connect returns the value of if the comPort is open. If it is already open it will just return true
        public bool Connect(string comPort, int baud = 115200)
        {
            if (serialPort == null)
            {
                serialPort = new SerialPort();
            }
            if (!serialPort.IsOpen)
            {
                serialPort.BaudRate = baud;
                serialPort.PortName = comPort;
                serialPort.ReadTimeout = 500;
                serialPort.StopBits = StopBits.One;
                serialPort.Parity = Parity.None;
                serialPort.DataBits = 8;
                serialPort.DataReceived += SerialPort_DataReceived;
                try
                {
                    serialPort.Open();
                    
                }
                catch
                {
                    serialPort?.Close();
                    _ConnectionStatus = connectionStatusEnum.Error;
                }
                
            }

            return serialPort.IsOpen;
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string message = "";
            try
            {
                message = serialPort.ReadLine();
                
            }
            catch (TimeoutException timeoutException)
            {

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.StackTrace);
            }


            if (!string.IsNullOrEmpty(message))
            {
                ConsoleOutput = message;
                if (OnSerialMessageReceived != null)
                {
                    OnSerialMessageReceived(this, new MessageEventArgs(message));//Invoke event that a new message was received
                }
                //NotifyPropertyChanged("ConsoleOutput");
            }
                

        }

        //Disconnect from the serial device
        public void Disconnect()
        {
            serialPort.Close();
            //updateTimer.Stop();
            _ConnectionStatus = connectionStatusEnum.Disconnected;
        }

        public int Write(string cmd)
        {
            try
            {
                serialPort.Write(cmd);
            }
            catch
            {
                return 0;
            }

            return cmd.Length;
        }

        public string Read(int timeout)
        {
            if(serialPort.ReadTimeout != timeout)
                serialPort.ReadTimeout = timeout;

            return serialPort.ReadLine();
        }

        //This will refresh the com ports for the drop down.
        private void updateComPorts()
        {
            ComPorts = new ObservableCollection<string>(SerialPort.GetPortNames());
        }

        //Just a testing method, will remove before release
        public void ChangeConnectionStatus(string connection)
        {
            ConnectionStatus = connection;
        }


    }
}
