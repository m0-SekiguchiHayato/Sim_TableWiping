


// hayato
// 元々はシミュレーションの描画に関する場所. ウインドウを一つに統一したため, 使用しなくなった. コメントアウトしただけで, コードの改変はしていない
// Simulator.xamlはエラーを吐かないよう色々消してある. 元々のコードはOpenRCF ver2.6をダウンロードすれば確認できる



/*
namespace RobotController
{
    /// <summary>
    /// Simulator.xaml の相互作用ロジック
    /// </summary>
    public partial class Simulator : Window
    {
        public Simulator()
        {
            InitializeComponent();
            glHost.Child = Camera.GLControl;
            base.Closing += WindowClosingEvent;
        }

        public new void Show()
        {
            if (base.IsActive == false) base.Show();
        }

        private void WindowClosingEvent(object sender, CancelEventArgs e)
        {
            e.Cancel = true;
            this.Visibility = Visibility.Collapsed;
        }

        private void ButtonRotate_Click(object sender, RoutedEventArgs e)
        {
            sliderRotate.Value = 0;
            Camera.Angle = 0;
        }

        private void SliderRotate_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Camera.Angle = (float)sliderRotate.Value;
        }

        private void ButtonZoom_Click(object sender, RoutedEventArgs e)
        {
            if (Camera.Height < 3) Camera.Height += 0.5f;
            else Camera.Height = 0;
        }

        private void SliderZoom_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Camera.Distance = Camera.DistanceInit + (float)sliderZoom.Value;
        }

        private void KeyDownHandler(object sender, KeyEventArgs e)
        {
            OpenRCF.Keyboard.KeyDownEvent(sender, e);
            if (OpenRCF.Keyboard.IsSpaceDowned) Camera.SetSubjectPosition(OpenRCF.Keyboard.SpaceVector.Get);
        }

        private void KeyUpHandler(object sender, KeyEventArgs e)
        {            
            OpenRCF.Keyboard.KeyUpEvent(sender, e);
        }

    }

    public partial class MainWindow : Window
    {
        Simulator Simulator = new Simulator();
        static float PI = (float)System.Math.PI;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += InitializeOpenRCF;
            Setup();
        }

        private void InitializeOpenRCF(object sender, RoutedEventArgs e)
        {
            Core.SetFPS(30);
            Core.SetDrawFunction = Draw;
            Simulator.Owner = this;
            Simulator.Show();
        }

        private void KeyDownHandler(object sender, KeyEventArgs e)
        {
            OpenRCF.Keyboard.KeyDownEvent(sender, e);
            if (OpenRCF.Keyboard.IsSpaceDowned) Camera.SetSubjectPosition(OpenRCF.Keyboard.SpaceVector.Get);
        }

        private void KeyUpHandler(object sender, KeyEventArgs e)
        {
            OpenRCF.Keyboard.KeyUpEvent(sender, e);
        }

    }
}
*/