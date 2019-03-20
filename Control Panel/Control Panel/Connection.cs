using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace Control_Panel
{
    class Connection : INotifyPropertyChanged
    {
        private enum connectionStatusEnum
        {
            Disconnected,
            Connected,
            Error
        }
        connectionStatusEnum connectionStatus;

        private string _connectionStatus;
        private ObservableCollection<string> comPorts;
        public event PropertyChangedEventHandler PropertyChanged;
        

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
            get { return connectionStatus; }
            set
            {
                connectionStatus = value;
                switch (value)
                {
                    case connectionStatusEnum.Connected:
                        ConnectionStatus = "Connected";
                        break;

                    case connectionStatusEnum.Disconnected:
                        ConnectionStatus = "Disconnected";
                        break;

                    case connectionStatusEnum.Error:
                        ConnectionStatus = "Error";
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

        private void NotifyPropertyChanged([CallerMemberName] String propertyName = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
        public Connection()
        {
            ConnectionStatus = "Disconnected";
            updateComPorts();
        }

        private void updateComPorts()
        {
            ComPorts = new ObservableCollection<string>(SerialPort.GetPortNames());
        }

        public void ChangeConnectionStatus(string connection)
        {
            ConnectionStatus = connection;
        }


    }
}
