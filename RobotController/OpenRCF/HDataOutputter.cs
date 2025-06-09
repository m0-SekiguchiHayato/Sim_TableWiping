using System.IO;

namespace OpenRCF
{
    public static class HDataOutputter    // データの出力に関する静的クラス
    {

        private static string filePathWithoutFileName = @"C:\Users\teamn\Desktop\Table Wiping (ver2.7)\Program\SavedDatum\";      // 武居研のパソコン用
        //private static string filePathWithoutFileName = @"C:\Users\brack\Desktop\Table Wiping (ver2.7)\Program\SavedDatum\";     // 自分のパソコン用

        private static bool isCurrentDisplayMode = false;    // 電流を表示するかどうか
        private static bool isCurrentSaveMode = false;    // 電流のデータを保存するかどうか

        // クラス外読み取り用
        public static bool IsCurrentDisplayMode { get { return isCurrentDisplayMode; } }
        public static bool IsCurrentSaveMode { get { return isCurrentSaveMode; } }


        // 電流の表示・非表示を切り替える関数
        public static void SwitchCurrentDisplayMode()
        {
            isCurrentDisplayMode = !isCurrentDisplayMode;
        }

        // 電流の保存をするかどうかを切り替える関数
        public static void SwitchCurrentSaveMode()
        {
            isCurrentSaveMode = !isCurrentSaveMode;
        }


        // ファイルに文字列の書き込みを行う関数
        public static void WriteFile_txt(string fileName, string content)
        {

            // trueは追記, falseは上書き
            using (StreamWriter SW = new StreamWriter(filePathWithoutFileName + fileName, true)) SW.WriteLine(content);

        }

        // ファイルにCSV形式のデータを書き込む関数
        public static void WriteFile_csv(string fileName, float[] datum)
        {

            // trueは追記, falseは上書き
            using (StreamWriter SW = new StreamWriter(filePathWithoutFileName + fileName, true))
            {

                for (int i = 0; i < datum.Length; i++)
                {
                    if (i != 0) SW.Write(", ");
                    SW.Write(datum[i]);
                }

                SW.WriteLine();

            }

        }

    }
}
