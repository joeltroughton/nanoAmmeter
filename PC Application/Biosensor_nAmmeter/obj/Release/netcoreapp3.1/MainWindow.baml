   M S B A M L   `   `   `  ���� M  IBiosensor_nAmmeter, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null7 clr-namespace:Biosensor_nAmmeterBiosensor_nAmmeter  U QScottPlot.WPF, Version=4.0.45.0, Culture=neutral, PublicKeyToken=e53b06131e34a3aa<.clr-namespace:ScottPlot;assembly=ScottPlot.WPF	ScottPlot #    Biosensor_nAmmeter.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35H 9http://schemas.microsoft.com/winfx/2006/xaml/presentation      R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     2local clr-namespace:Biosensor_nAmmeter     D	ScottPlot.clr-namespace:ScottPlot;assembly=ScottPlot.WPF       � Title$6  0Biosensor nanoAmmeter - v1 [Joel Troughton 2020]��$	��500��$	��800��.���  � ColumnDefinitions �� $��* ��� $��3* �.+���  �� Padding$ 10q�._��� $��0��$��0,0,5,0q�.+��� $��Select port��$
��Bold"�$��0,0,0,0q��� -   $��portsbox�� �� Text$ Select port�� �� SelectedIndex$ 1���  �� $��* ��� $��* �.+��� -    ��Name$ connect��$��0��$��Connect���� -   $ 
disconnect��$��1��$��
Disconnect���� -    ��Name$ status��$��23�� �� TextWrapping$
 Wrap=�$��Disconnected���� $��Measurement control��$
��Bold"�$��0,10,0,0q��  �� $��* ��� $��* ��� $��* �.+��� -   $ start_measurement��$��0��$��Start���� -   $ stop_measurement��$��1��$
��Stop���� -   $ single_measurement��$��2��$��Single���� -   $ measurement_status��$��23��$
 Wrap=�$��Not running���� $��
Biosensor ��$
��Bold"�$��0,10,0,0q��  �� $��* ��� $��* �.+��� $��0��$��PV voltage (V)��$��0,0,0,0q��� $��1��$��Current (nA)��$��0,0,0,0q��  �� $��* ��� $��* �.+��� -	   $ voltage��$��0��$��23��$
 Wrap=�$��0���� -
   $ current��$��1��$��23��$
 Wrap=�$��0���� $��Data export ��$
��Bold"�$��0,10,0,0q��  �� $��4* ��� $��* �.+��� -   $ saveDirectoryText��$��23��$��0,0,0,0q�$��0��$
 Wrap=�$��C:\data��$��Center=�$
{�True���� -   $ saveDirectory��$��Select��$��0,0,0,0q�$��1���� -   $ export��$��Export data���� -   $ clear��$��Clear graph��]� $��1��$
��Auto��$��Stretch=�$��Stretch=�.+��  � RowDefinitions �� $��2* ��� $��* �.+�  ScottPlot.WpfPlot  -   $��wpfPlot1��$��0��  -   $��wpfPlot2��$��1��