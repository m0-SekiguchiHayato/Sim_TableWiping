# はじめに
本シミュレーションは、フリーソフト「OpenRCF」を用いて作成したものです。使用したバージョンは2.7で、次ののURLからダウンロード可能です（2025年6月10日現在）。\
https://mase.openrcf.site/

## シミュレーションの実行
１．「master」ブランチのzipファイルをダウンロードし、解凍してください。\
２．解凍後、フォルダ内の「TableWiping.sln」を実行してください。Visual Studioが起動します。\
３．「開始」をクリックするとシミュレーションが起動します。\
「MainWindow.xaml.cs」が主なシミュレーション編集ファイルとなります。ここからウインドウに表示されるボタンへの関数割当が可能です。

## ソースコードの閲覧
ソースコードのみを閲覧したい場合には、「master」ブランチのRobotController\OpenRCFをご覧ください。ここに含まれるcsファイルのうち、ファイル名が「H」から始まるものは私が作成したクラスです。その他のcsファイルでも、「hayato」とコメントアウトがある部分は私がオリジナルから編集した部分です。また、シミュレーションの編集のため、RobotControllerフォルダ内の「MainWindow.xaml」「MainWindow.xaml.cs」「Simulator.xaml」「Simulator.xaml.cs」の４つにも手を加えています。

## OpenRCFの基本的な使い方について
「main」ブランチ内にある「OpenRCF ver2.7 説明書.pdf」にて、本ソフトのオリジナルに備わっている機能が紹介されています。
