using System;
using System.Windows.Threading;

namespace OpenRCF
{
    public static class Core
    {
        private static DispatcherTimer timer = new DispatcherTimer(DispatcherPriority.Normal);
        private static EventHandler eventHandler;
        private static uint FPS = 30;

        public static Rectangle Tile = new Rectangle(6, 6);
        public static ThreeAxis ReferenceFrame = new ThreeAxis(0.1f);

        static Core()
        {
            timer.Tick += eventHandler;
            Tile.Color.SetRGB(225, 225, 225);
            Tile.Color.Alpha = 160;
        }

        public static Action SetDrawFunction
        {
            set
            {
                timer.Tick -= eventHandler;
                eventHandler = (sender, e) =>
                {
                    Tile.DrawLineNet(11);
                    ReferenceFrame.Draw();
                    value();
                    Tile.Draw();
                    Camera.DisplayUpdate();
                };
                timer.Tick += eventHandler;
                timer.Interval = TimeSpan.FromMilliseconds(1000 / FPS);
                timer.Start();
            }
        }

        public static void SetFPS(uint FPS)
        {
            if (0 < FPS && FPS <= 50)
            {
                Core.FPS = FPS;
                timer.Interval = TimeSpan.FromMilliseconds(1000 / FPS);
            }
            else
            {
                Console.WriteLine("Error : Max FPS is 50.");
            }
        }
    }

}
