# 高崎高校物理部2年 2022-2023 LightWeight
高崎高校物理部2年チームのGithubリポジトリです。このリポジトリで公開されているすべてのデータは自由に使ってもらって構いません。  
各種リンク
* [ツイッター](https://twitter.com/takataka_robo)
* [ブログ](https://robocup-zunda.hatenablog.com)
## リポジトリの構成
以下に、このリポジトリのディレクトリ構造をのせておきます。  

2022-2023-LightWeight/  
&emsp;├ Hardware/  
&emsp;│└ N号機/  
&emsp;│&emsp;├ PCB/&emsp;&emsp;&emsp;※基板設計データ  
&emsp;│&emsp;└ STL/&emsp;&emsp;&emsp;※３D部品データ   
&emsp;│  
&emsp;└ SoftWare/  
&emsp;&emsp;├ Archived/&emsp;&emsp;&emsp;※開発時のデータ（未使用）  
&emsp;&emsp;└ Robot Gen N/&emsp;&emsp;&emsp;※Rev.Nの機体のプログラム  

## ロボットの機体の概要
今年のロボットの3Dデータです。-> https://a360.co/3iR0CL1  
以下に、今年度のロボットの基本的なスペックについて示します。

* Main  
  * Raspberry Pi PICO1個
  * MPU-6050 １個
  * SSD1306 OLEDディスプレイ 1個
  * 【予定】XIAO ESP32C3 1個
* IR Unit
  * TSSP58038 16個
  * TI CD74HC4067 SOIC 1個
  * XIAO SAMD21 1個
* Actuator
  * Maxon RE16 4個
  * Maxon GP16 1:19 4個
  * Pololu G2 18v17 4個
  * 自作オムニホイール 4個
  * XIAO RP2040 1個
* Line Unit
  * NJL7502L 32個
  * WS2812B 32個
  * TI CD74HC4067 SOIC 2個
* Cam
  * OpenMV H7 R2 1個
  * 自作双曲線ミラー 1個
* Kicker
  * 自作DCDCコンバータ 1個  
&emsp;※XL6009採用 Max 50V
  * タカハ CB1034 1個
  
## コーディング規則
簡単なコーディング規則を定めておきます。  
### ファイル構成
プログラムの中核部分を `main.cpp`に書くようにしてください。初見でプログラムを見た際にこのプログラムで大方何をやっているかわかるようにすることを意識してください。`setup()`と`loop()`は他のファイルへ移すことは禁止します。特定の機能にたくさんの記述が必要な場合にはライブラリにしてインクルードしてください。その際、インクルードガードをなるべくつけるようにしましょう。

### 変数・関数・クラス
それぞれの名前は、追加情報なしでもおおよそ役割が推測できる名前にしてくだいさい。例えば、`flag`などといいう名前では、なんのフラグなのかわからないので、`line_flag`などわかりやすい配慮をしてください。定数は、原則大文字で命名してください。  
各ユニットごとに、クラスを作成するようにして下さい。特定のユニットに依存した変数や関数はグローバルで作成するのではなく、各クラスのインスタンスにしましょう。インスタンスはできる限り`private`に閉じ込めることが望ましいです。

##コンタクト
もし、なにかありましたらチームリーダへ連絡していただけると幸いです。  
Mail💌&emsp;nkoji.personal@gmail.com&emsp;&emsp;[Twitter](https://twitter.com/negi_robo)