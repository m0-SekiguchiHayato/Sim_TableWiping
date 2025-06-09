using System;
using System.Threading;
using System.Windows.Threading;

namespace OpenRCF
{
    public static class Parallel
    {
        private static Task[] task = new Task[10];

        static Parallel()
        {
            for (int i = 0; i < task.Length; i++)
            {
                task[i] = new Task();
            }
        }

        public static void Run(Action timerEvent, uint timeOutMs, uint timeSpanMs)
        {
            for (int i = 0; i < task.Length; i++)
            {
                if (task[i].IsRunning == false)
                {
                    task[i].TimeOutUs = 1000 * timeOutMs;
                    task[i].IsStop = () => false;
                    task[i].TimeSpanUs = 1000 * timeSpanMs;
                    task[i].Run(timerEvent);
                    return;
                }
                else if (task[i].IsSameEvent(timerEvent))
                {
                    Console.WriteLine("Error : Same timerEvent is already running.");
                    return;
                }
                else if (i == task.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of Parallel.Run() is " + task.Length);
                }
            }
        }

        public static void Run(Action timerEvent, Func<bool> isStop, uint timeSpanMs)
        {
            for (int i = 0; i < task.Length; i++)
            {
                if (task[i].IsRunning == false)
                {
                    task[i].TimeOutUs = long.MaxValue;
                    task[i].IsStop = isStop;
                    task[i].TimeSpanUs = 1000 * timeSpanMs;
                    task[i].Run(timerEvent);
                    return;
                }
                else if (task[i].IsSameEvent(timerEvent))
                {
                    Console.WriteLine("Error : Same timerEvent is already running.");
                    return;
                }
                else if (i == task.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of Parallel.Run() is " + task.Length);
                }
            }
        }

        public static void Run(Func<bool> timerEvent, uint timeSpanMs)
        {
            for (int i = 0; i < task.Length; i++)
            {
                if (task[i].IsRunning == false)
                {
                    task[i].TimeOutUs = long.MaxValue;
                    task[i].IsStop = () => false;
                    task[i].TimeSpanUs = 1000 * timeSpanMs;
                    task[i].Run(timerEvent);
                    return;
                }
                else if (task[i].IsSameEvent(timerEvent))
                {
                    Console.WriteLine("Error : Same timerEvent is already running.");
                    return;
                }
                else if (i == task.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of Parallel.Run() is " + task.Length);
                }
            }
        }

        public static void RunEndless(Action timerEvent, uint timeSpanMs)
        {
            for (int i = 0; i < task.Length; i++)
            {
                if (task[i].IsRunning == false)
                {
                    task[i].TimeOutUs = long.MaxValue;
                    task[i].IsStop = () => false;
                    task[i].TimeSpanUs = 1000 * timeSpanMs;
                    task[i].Run(timerEvent);
                    return;
                }
                else if (task[i].IsSameEvent(timerEvent))
                {
                    Console.WriteLine("Error : Same timerEvent is already running.");
                    return;
                }
                else if (i == task.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of Parallel.Run() is " + task.Length);
                }
            }
        }

        public static void RunWait(Action task, int waitTimeMs)
        {
            System.Threading.Tasks.Task.Run(() => { Thread.Sleep(waitTimeMs); task(); });
        }

        public static void RunWait(Action task, Func<bool> canStart)
        {
            System.Threading.Tasks.Task.Run(() => { while (!canStart()) { Thread.Sleep(50); }; task(); });
        }


        // hayato：canStartがtrueを返してからtask()を実行するまでの時間に, インターバルを設けるRunWait
        public static void RunWait(Action task, bool canStart, int intervalTimeMs)
        {
            System.Threading.Tasks.Task.Run(() =>
            {
                while (!canStart) { Thread.Sleep(50); };
                Thread.Sleep(intervalTimeMs);
                task();
            });
        }


        private class Task
        {
            private TimeStamp timeStamp = new TimeStamp();
            public bool IsRunning = false;
            public long TimeOutUs = 5000;
            public long TimeSpanUs = 1000;
            public Func<bool> IsStop = () => false;
            private int sleepTimeMs;

            private Action TimerEvent;
            public bool IsSameEvent(Action timerEvent) { return TimerEvent.Equals(timerEvent); }

            private Func<bool> BoolTimerEvent = () => { return false; };
            public bool IsSameEvent(Func<bool> timerEvent) { return BoolTimerEvent.Equals(timerEvent); }

            public void Run(Action timerEvent)
            {
                IsRunning = true;
                uint count = 0;
                timeStamp.Set();
                TimerEvent = timerEvent;

                System.Threading.Tasks.Task.Run(() =>
                {
                    while (true)
                    {
                        if (count * TimeSpanUs <= timeStamp.ElapsedTimeUs)
                        {
                            if (TimeOutUs <= timeStamp.ElapsedTimeUs || IsStop())
                            {
                                break;
                            }
                            else
                            {
                                TimerEvent();
                                count++;
                                sleepTimeMs = (int)((count * TimeSpanUs - timeStamp.ElapsedTimeUs) / 1000);
                                if (0 < sleepTimeMs) Thread.Sleep(sleepTimeMs - 1);
                            }
                        }
                    }

                    IsRunning = false;
                    TimerEvent = () => { };
                });
            }

            public void Run(Func<bool> timerEvent)
            {
                IsRunning = true;
                uint count = 0;
                timeStamp.Set();
                BoolTimerEvent = timerEvent;

                System.Threading.Tasks.Task.Run(() =>
                {
                    while (true)
                    {
                        if (count * TimeSpanUs <= timeStamp.ElapsedTimeUs)
                        {
                            if (BoolTimerEvent())
                            {
                                count++;
                                sleepTimeMs = (int)((count * TimeSpanUs - timeStamp.ElapsedTimeUs) / 1000);
                                if (0 < sleepTimeMs) Thread.Sleep(sleepTimeMs - 1);
                            }
                            else
                            {
                                break;
                            }
                        }
                    }

                    IsRunning = false;
                    BoolTimerEvent = () => { return true; };
                });
            }

            private class TimeStamp
            {
                private long timeStamp = DateTime.Now.Ticks;

                public void Set()
                {
                    timeStamp = DateTime.Now.Ticks;
                }

                public long ElapsedTimeUs
                {
                    get { return (long)((DateTime.Now.Ticks - timeStamp) / 10); }
                }
            }
        }

    }

    public static class ParallelUI
    {
        private static EventTimer[] timer = new EventTimer[10];

        static ParallelUI()
        {
            for (int i = 0; i < timer.Length; i++)
            {
                timer[i] = new EventTimer();
            }
        }

        public static void Run(Action timerEvent, uint timeOutMs, uint timeSpanMs)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (timer[i].IsEnabled == false)
                {
                    timer[i].EventHandler = (sender, e) =>
                    {
                        if (timer[i].TimeStamp.ElapsedTimeMs < timeOutMs) timerEvent();
                        else timer[i].Stop();
                    };
                    timer[i].Interval = TimeSpan.FromMilliseconds(timeSpanMs);
                    timer[i].TimeStamp.Set();
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        public static void Run(Action timerEvent, Func<bool> isStop, uint timeSpanMs)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (timer[i].IsEnabled == false)
                {
                    timer[i].EventHandler = (sender, e) =>
                    {
                        if (isStop()) timer[i].Stop();
                        else timerEvent();
                    };
                    timer[i].Interval = TimeSpan.FromMilliseconds(timeSpanMs);
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        public static void Run(Func<bool> timerEvent, uint timeSpanMs)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (timer[i].IsEnabled == false)
                {
                    timer[i].EventHandler = (sender, e) => { if (!timerEvent()) timer[i].Stop(); };
                    timer[i].Interval = TimeSpan.FromMilliseconds(timeSpanMs);
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        public static void RunEndless(Action timerEvent, uint timeSpanMs)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (timer[i].IsEnabled == false)
                {
                    timer[i].EventHandler = (sender, e) => { timerEvent(); };
                    timer[i].Interval = TimeSpan.FromMilliseconds(timeSpanMs);
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        public static void RunWait(Action timerEvent, uint waitTimeMs)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (!timer[i].IsEnabled)
                {
                    timer[i].EventHandler = (sender, e) =>
                    {
                        timerEvent();
                        timer[i].Stop();
                    };
                    timer[i].Interval = TimeSpan.FromMilliseconds(waitTimeMs);
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        public static void RunWait(Action timerEvent, Func<bool> canStart)
        {
            for (int i = 0; i < timer.Length; i++)
            {
                if (!timer[i].IsEnabled)
                {
                    timer[i].EventHandler = (sender, e) =>
                    {
                        if (canStart())
                        {
                            timerEvent();
                            timer[i].Stop();
                        }
                    };
                    timer[i].Interval = TimeSpan.FromMilliseconds(50);
                    timer[i].Start();
                    return;
                }
                else if (i == timer.Length - 1)
                {
                    Console.WriteLine("Error : Maximum number of ParallelUI.Run() is " + timer.Length);
                }
            }
        }

        private class TimeStamp
        {
            private long timeStamp = DateTime.Now.Ticks;

            public void Set()
            {
                timeStamp = DateTime.Now.Ticks;
            }

            public long ElapsedTimeMs
            {
                get { return (long)((DateTime.Now.Ticks - timeStamp) / 10000); }
            }
        }

        private class EventTimer : DispatcherTimer
        {
            private EventHandler eventHandler;
            public TimeStamp TimeStamp = new TimeStamp();

            public EventTimer() : base(DispatcherPriority.Normal)
            {
                Tick += eventHandler;
            }

            public EventHandler EventHandler
            {
                set
                {
                    Tick -= eventHandler;
                    eventHandler = value;
                    Tick += eventHandler;
                }
            }
        }

    }

    public static class Timer
    {
        private static long[] timeLabel = new long[20];

        static Timer()
        {
            long timeStamp = DateTime.Now.Ticks;

            for (int i = 0; i < timeLabel.Length; i++)
            {
                timeLabel[i] = timeStamp;
            }
        }

        public static void SetTimeLabel(uint labelNum)
        {
            if (labelNum < timeLabel.Length) timeLabel[labelNum] = DateTime.Now.Ticks;
            else Console.WriteLine("Error : Max of Time Labels is " + timeLabel.Length);
        }

        public static long GetElapsedTimeMs(uint labelNum)
        {
            return (DateTime.Now.Ticks - timeLabel[labelNum]) / 10000;
        }

        public static float GetElapsedTimeSec(uint labelNum)
        {
            return (DateTime.Now.Ticks - timeLabel[labelNum]) / 10000000f;
        }

    }

}
