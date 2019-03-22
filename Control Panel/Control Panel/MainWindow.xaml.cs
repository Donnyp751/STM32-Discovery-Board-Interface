using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading;
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
using System.Windows.Threading;
using OxyPlot;
using OxyPlot.Wpf;

namespace Control_Panel
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private DispatcherTimer updateTimer = new DispatcherTimer();
        private Connection connection;
        private double time = 0;
        public ObservableCollection<DataPoint> TemperatureData
        {
            get { return (ObservableCollection<DataPoint>)GetValue(DataProperty); }
            set { SetValue(DataProperty, value); }
        }
        public ObservableCollection<DataPoint> HumidityData
        {
            get { return (ObservableCollection<DataPoint>)GetValue(DataProperty); }
            set { SetValue(DataProperty, value); }
        }
        public MainWindow()
        {
            InitializeComponent();
            connection = new Connection();
            MainGrid.DataContext = connection;
            ConsoleTb.TextChanged += ConsoleOutput_TextChanged;
            updateTimer.Interval = TimeSpan.FromMilliseconds(1000);
            connection.OnSerialMessageReceived += Connection_OnSerialMessageReceived;

        }

        private void parseTempAndHumid(string message, out float temp, out float humid)
        {
            string[] splitMessage = message.Split('\t');
            temp = 0;
            humid = 0;
            foreach (string part in splitMessage)
            {
                if (part.Contains("temperature"))
                {
                    string tempStr = part.Remove(0, 21);
                    tempStr = tempStr.TrimEnd(new char[] {'C', ' '});

                    temp = (float) Convert.ToDouble(tempStr);
                }else if (part.Contains("humidity"))
                {
                    string humidStr = part.Remove(0, 16);
                    humidStr = humidStr.TrimEnd(new char[] {'R', 'H', ' '});
                    humid = (float) Convert.ToDouble(humidStr);
                }
            }
        }
        private void Connection_OnSerialMessageReceived(object sender, MessageEventArgs args)
        {
            parseTempAndHumid(args.Message, out float temp,out float humid );
            TemperatureData.Add(new DataPoint(time, temp));
            HumidityData.Add(new DataPoint(time, humid));
        }

        private void ConsoleOutput_TextChanged(object sender, TextChangedEventArgs e)
        {
            ConsoleTb.ScrollToEnd();
        }

        private void ConnectBtnClick(object sender, RoutedEventArgs e)
        {
            string comPort = ComPortsComboBox?.SelectionBoxItem.ToString();
            
            if(!connection.IsOpen)
            {
                if (!string.IsNullOrEmpty(comPort))
                {
                    if (connection.Connect(comPort))
                        ConnectButton.Content = "Disconnect";
                }
            }
            else
            {
                connection.Disconnect();
                ConnectButton.Content = "Connect";
            }
        }
        public static readonly DependencyProperty DataProperty =
            DependencyProperty.Register("Data", typeof(ObservableCollection<DataPoint>), typeof(MainWindow), new PropertyMetadata(new ObservableCollection<DataPoint>()));


    }
}
