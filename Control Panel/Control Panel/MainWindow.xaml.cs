using System;
using System.Collections.Generic;
using System.ComponentModel;
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

namespace Control_Panel
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Connection connection;
        public MainWindow()
        {
            InitializeComponent();
            connection = new Connection();

            MainGrid.DataContext = connection;
            //ConnectionStatusTextBlock.DataContext = connection;
           
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
    }
}
