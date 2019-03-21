﻿using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Timers;

namespace Control_Panel
{
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

        public ObservableCollection<string> ConsoleOutput
        {
            get => consoleOutput;
            set
            {
                if (value != null)
                {
                    consoleOutput = value;
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

            _ConnectionStatus = connectionStatusEnum.Disconnected;
            updateComPorts();
        }

        private void TimedUpdate(object sender, ElapsedEventArgs e)
        {
            if (serialPort == null) return;


            IsOpen = serialPort.IsOpen;
            if (serialPort.IsOpen)
            {
                _ConnectionStatus = connectionStatusEnum.Connected;
                string message = "";
                try
                {
                    message = serialPort.ReadLine();
                }
                catch
                {

                }

                if (!string.IsNullOrEmpty(message))
                    ConsoleOutput?.Add(message);
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
                try
                {
                    serialPort.Open();
                    updateTimer.Start();
                }
                catch
                {
                    serialPort?.Close();
                    _ConnectionStatus = connectionStatusEnum.Error;
                }
                
            }

            return serialPort.IsOpen;
        }

        //Disconnect from the serial device
        public void Disconnect()
        {
            serialPort.Close();
            updateTimer.Stop();
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

        public string Read()
        {
            return serialPort.ReadLine();
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
