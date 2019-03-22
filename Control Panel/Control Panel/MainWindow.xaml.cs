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
        public static readonly DependencyProperty TemperatureDataProperty =
            DependencyProperty.Register("TemperatureData", typeof(ObservableCollection<DataPoint>), typeof(MainWindow), new PropertyMetadata(new ObservableCollection<DataPoint>()));
        public static readonly DependencyProperty HumidityDataProperty =
            DependencyProperty.Register("HumidityData", typeof(ObservableCollection<DataPoint>), typeof(MainWindow), new PropertyMetadata(new ObservableCollection<DataPoint>()));

        private float latestTemperatureReading = 0;
        private float latestHumidityReading = 0;
        private DispatcherTimer updateTimer = new DispatcherTimer();
        private Connection connection;
        private double time = 0;
        public ObservableCollection<DataPoint> TemperatureData
        {
            get { return (ObservableCollection<DataPoint>)GetValue(TemperatureDataProperty); }
            set { SetValue(TemperatureDataProperty, value); }
        }
        public ObservableCollection<DataPoint> HumidityData
        {
            get { return (ObservableCollection<DataPoint>)GetValue(HumidityDataProperty); }
            set { SetValue(HumidityDataProperty, value); }
        }
        public MainWindow()
        {
            InitializeComponent();
            connection = new Connection();
            MainGrid.DataContext = connection;
            ConsoleTb.TextChanged += ConsoleOutput_TextChanged;
            SensorDataDisplayGrid.DataContext = this;
            updateTimer.Interval = TimeSpan.FromMilliseconds(1000);
            updateTimer.Tick += UpdateTimer_Tick;

            connection.OnSerialMessageReceived += Connection_OnSerialMessageReceived;

        }

        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            TemperatureData.Add(new DataPoint(time, latestTemperatureReading));
            HumidityData.Add(new DataPoint(time, latestHumidityReading));
            time++;
        }

        private char[] numbers = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '.'};
        private void parseTempAndHumid(string message, out float temp, out float humid)
        {
            string[] splitMessage = message.Split('\t');
            temp = 0;
            humid = 0;
            foreach (string part in splitMessage)
            {
                if (part.Contains("temperature"))
                {
                    var splitEnd = part.IndexOfAny(numbers, part.Length/2);
                    string tempStr = part.Remove(0, splitEnd);

                    //tempStr = part.Remove(0, part.IndexOfAny(numbers,0)-1);
                    tempStr = tempStr.Remove(tempStr.LastIndexOfAny(numbers),
                       tempStr.Length - tempStr.LastIndexOfAny(numbers));

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
            latestTemperatureReading = temp;
            latestHumidityReading = humid;
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
                    {
                        updateTimer.Start();
                        ConnectButton.Content = "Disconnect";
                    }
                }
            }
            else
            {
                updateTimer.Stop();
                connection.Disconnect();
                ConnectButton.Content = "Connect";
                
            }
        }


    }
}
