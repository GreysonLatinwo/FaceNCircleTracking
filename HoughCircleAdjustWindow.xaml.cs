﻿using System;
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

namespace Microsoft.Samples.Kinect.FaceBasics
{
    /// <summary>
    /// Interaction logic for HoughCircleAdjustWindow.xaml
    /// </summary>
    public partial class HoughCircleAdjustWindow : Window
    {
        public event EventHandler<double> CircleDetectionAdjusted;
        public HoughCircleAdjustWindow()
        {
            InitializeComponent();
        }
        public virtual void Slider_ValueChanged_0(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            //invoke the event handler for the circle event
            Console.WriteLine((sender as Slider).Name);
            CircleDetectionAdjusted?.Invoke((sender as Slider), e.NewValue);
        }
    }
}
