using System;
using System.IO;
using System.IO.Ports;
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
using System.Diagnostics;
using ScottPlot.plottables;
using ScottPlot;

using System.Windows.Threading;

namespace Biosensor_nAmmeter
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        SerialPort sp = new SerialPort();
        public string[] ports = SerialPort.GetPortNames();

        public double[] timestampdata = new double[100000];
        public double[] currentdata = new double[100000];
        public double[] voltagedata = new double[100000];

        public int nextDataIndex = 0;
        public int voltagenextDataIndex = 0;

        Stopwatch stopwatch = new Stopwatch();


        PlottableSignal currentPlot;
        PlottableSignal voltagePlot;

        PlottableVLine vLine;
        PlottableHLine hLine;




        public MainWindow()
        {
            InitializeComponent();
            portsbox.ItemsSource = ports;

            currentPlot = wpfPlot1.plt.PlotSignal(currentdata, useParallel: true, lineWidth: 2);
            wpfPlot1.plt.YLabel("Biosensor current (nA)");
            wpfPlot1.plt.XLabel("Data point");

            voltagePlot = wpfPlot2.plt.PlotSignal(voltagedata, useParallel: true, lineWidth: 2);
            wpfPlot2.plt.YLabel("PV Voltage (mV)");
            wpfPlot2.plt.XLabel("Data point");

            //vLine = wpfPlot1.plt.PlotVLine(0, color: System.Drawing.Color.Red, lineStyle: LineStyle.Dash);
            //hLine = wpfPlot1.plt.PlotHLine(0, color: System.Drawing.Color.Red, lineStyle: LineStyle.Dash);

            DispatcherTimer renderTimer = new DispatcherTimer();
            renderTimer.Interval = TimeSpan.FromMilliseconds(16);
            renderTimer.Tick += Render;
            renderTimer.Start();

            disconnect.IsEnabled = false;
            start_measurement.IsEnabled = false;
            stop_measurement.IsEnabled = false;
            single_measurement.IsEnabled = false;
            export.IsEnabled = false;
            clear.IsEnabled = false;
            saveDirectory.IsEnabled = false;
        }

        private void Connect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                string portName = portsbox.Text;
                sp.PortName = portName;
                sp.BaudRate = 115200;
                sp.DtrEnable = true;
                sp.RtsEnable = true;
                sp.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
                sp.Open();
                status.Text = "Connected to " + portName;
                disconnect.IsEnabled = true;
                connect.IsEnabled = false;
                portsbox.IsEnabled = false;
                disconnect.IsEnabled = true;
                start_measurement.IsEnabled = true;
                stop_measurement.IsEnabled = true;
                single_measurement.IsEnabled = true;
                export.IsEnabled = true;
                clear.IsEnabled = true;
                saveDirectory.IsEnabled = true;
            }
            catch (Exception)
            {
                MessageBox.Show("Connection failed. Is the correct port selected?");
            }
        }


        private void Disconnect_Click(object sender, RoutedEventArgs e)
        {
            sp.Close();
            disconnect.IsEnabled = false;
            connect.IsEnabled = true;
            portsbox.IsEnabled = true;

            status.Text = "Disconnected";
        }

        private void Portsbox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void stop_measurement_Click(object sender, RoutedEventArgs e)
        {
            string sendString = "";
            sendString = String.Format("stop");
            sp.WriteLine(sendString);
            Debug.Print("Stopping continuous measurement...");
            measurement_status.Text = "Not running";
            measurement_status.Foreground = Brushes.Red;



        }

        private void start_measurement_Click(object sender, RoutedEventArgs e)
        {
            stopwatch.Stop();
            stopwatch.Reset();

            string sendString = "";
            sendString = String.Format("start");
            sp.WriteLine(sendString);
            Debug.Print("Starting continuous measurement...");
            measurement_status.Text = "Measurement running...";
            measurement_status.Foreground = Brushes.Green;
            stopwatch.Start();
        }

        private void single_measurement_Click(object sender, RoutedEventArgs e)
        {
            string sendString = "";
            sendString = String.Format("single");
            sp.WriteLine(sendString);
            Debug.Print("Requesting single measurement...");
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            string inLine = sp.ReadLine();
            //Dispatcher.Invoke(DispatcherPriority.Send, new UpdateUiTextDelegate(UpdateData), inLine);
            Dispatcher.Invoke(() =>
            {
                try
                {
                    UpdateData(inLine);
                    //replyRecieved = true;

                }
                catch (Exception ex)
                {
                    //  MessageBox.Show("Error interpreting input data");
                    //  Console.WriteLine("UpdateData error - {0}", ex);
                }
            });
        }

        private async void UpdateData(string inLine)
        {
            string str = inLine.Substring(0, inLine.Length);
            str = str.Replace("\0", string.Empty);

            try
            {
                string[] parts = str.Split(',');

                voltage.Text = parts[0];
                current.Text = parts[1];

                double dvoltage = double.Parse(voltage.Text);
                double dcurrent = double.Parse(parts[1]);

                currentdata[nextDataIndex] = dcurrent;
                currentPlot.maxRenderIndex = nextDataIndex;

                voltagedata[nextDataIndex] = dvoltage;
                voltagePlot.maxRenderIndex = voltagenextDataIndex;

                timestampdata[nextDataIndex] = (double)stopwatch.ElapsedMilliseconds / 1000;

                nextDataIndex += 1;
                voltagenextDataIndex += 1;

                Array.Clear(parts, 0, parts.Length);
            }

            catch (Exception ex)
            {
                Debug.Print("Error - Data from SMU not understood. {0}", ex);
            }
        }
        void Render(object sender, EventArgs e)
        {
            wpfPlot1.plt.AxisAuto(horizontalMargin: 0, verticalMargin: 0.25);
            wpfPlot1.Render(skipIfCurrentlyRendering: true);

            wpfPlot2.plt.AxisAuto(horizontalMargin: 0, verticalMargin: 0.25);
            wpfPlot2.Render(skipIfCurrentlyRendering: true);

        }


        private void export_Click(object sender, RoutedEventArgs e)
        {
            String filename = saveDirectoryText.Text;
            filename += "/";
            filename += DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss");
            string dataout = filename + ".csv";

            double[] timestampexport = new double[nextDataIndex];
            double[] voltageexport = new double[nextDataIndex];
            double[] currentexport = new double[nextDataIndex];

            timestampexport = timestampdata;
            voltageexport = voltagedata;
            currentexport = currentdata;

            Array.Resize(ref timestampexport, nextDataIndex);
            Array.Resize(ref voltageexport, nextDataIndex);
            Array.Resize(ref currentexport, nextDataIndex);


            using (StreamWriter file = new StreamWriter(dataout))
            {

                file.WriteLine("Time (ms), PV cell voltage (mV), Biosensor current (nA)");


                for (int i = 0; i < nextDataIndex; i++)
                {
                    file.Write(timestampexport[i] + ",");
                    file.Write(voltageexport[i] + ",");
                    file.WriteLine(currentexport[i] + ",");
                }

            }

            //SaveArrayAsCSV(currentdata, dataout, nextDataIndex);
        }

        private void clear_Click(object sender, RoutedEventArgs e)
        {
            Array.Clear(timestampdata, 0, timestampdata.Length);
            Array.Clear(currentdata, 0, currentdata.Length);
            Array.Clear(voltagedata, 0, voltagedata.Length);
            nextDataIndex = 0;
            voltagenextDataIndex = 0;
        }

        private void SaveDirectory_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new Microsoft.Win32.SaveFileDialog();

            dialog.InitialDirectory = saveDirectoryText.Text; // Use current value for initial dir
            dialog.Title = "Select a Directory"; // instead of default "Save As"
            dialog.Filter = "Directory|*.this.directory"; // Prevents displaying files
            dialog.FileName = "select"; // Filename will then be "select.this.directory"
            if (dialog.ShowDialog() == true)
            {
                string path = dialog.FileName;
                // Remove fake filename from resulting path
                path = path.Replace("\\select.this.directory", "");
                path = path.Replace(".this.directory", "");
                // If user has changed the filename, create the new directory
                if (!System.IO.Directory.Exists(path))
                {
                    System.IO.Directory.CreateDirectory(path);
                }
                // Our final value is in path
                saveDirectoryText.Text = path;
            }


        }

    }
}
