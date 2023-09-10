# ros2-example
`ros2-example`はROS2(Robot Operating System)のサンプル・コード集になります。  
コードの内容については、僕が運営している下記サイトに解説記事がありますので併せて参照してください。

- [たっきんの秘密の投資開発部屋](https://takilog.com/category/programming/robot-operating-system/)


## 実行環境
本サンプル・コードを実行するにはROS2の実行環境が必要になります。  
以下の記事を参考にしてROS2実行環境を構築しましょう！  

- [UbuntuにROS×Pythonの開発環境を導入しよう！](https://takilog.com/introduce-ubuntu-ros-python-environment/)


## ビルド＆実行
本リポジトリをクローンしてビルドします。

1. ワークスペースを作成します。
```bash
$ mkdir -p ~/git/
$ cd ~/git
```

2. 本リポジトリをクローンします。
```bash
$ git clone https://github.com/takkin-takilog/ros2-example.git
```

3. ビルドします。
```bash
$ cd ros2-example
$ colcon build --symlink-install
```

4. ワークスペースの環境設定  
ワークスペース情報を読み込みます。  
下記コマンドは新しい端末を開く度に実行してください。
```bash
$ source install/local_setup.bash
```

5. ノードを実行します。
```bash
$ ros2 run ros2_example <executable_name>
```
