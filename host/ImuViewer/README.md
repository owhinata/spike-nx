# ImuViewer

SPIKE Prime Hub の `btsensor` (Issue #56) が SPP/RFCOMM で配信する LSM6DSL の IMU
ストリームを PC 側で受信し、Madgwick filter で姿勢推定して 3D で可視化するデスクトップ
アプリ (Issue #60)。

- フレームワーク: .NET 10 / Avalonia 11.x / Silk.NET
- 対象 OS (PoC): Linux (BlueZ, BTPROTO_RFCOMM)
- macOS / Windows: 別 Issue で後追い (現状は `PlatformNotSupportedException`)

## プロジェクト構成

| プロジェクト | 役割 |
| --- | --- |
| `ImuViewer.Core` | フレームパース、Madgwick、座標変換、Bluetooth トランスポート抽象 |
| `ImuViewer.Rendering` | Silk.NET OpenGL で Cube + ワールド軸 + グリッドを描画 |
| `ImuViewer.App` | Avalonia UI (RViz 風レイアウト) |
| `ImuViewer.Core.Tests` | xUnit ベースの単体テスト |

## ビルドと実行 (Linux)

```bash
cd host/ImuViewer
dotnet restore ImuViewer.slnx
dotnet build   ImuViewer.slnx -c Debug
dotnet test    tests/ImuViewer.Core.Tests/ImuViewer.Core.Tests.csproj
dotnet run --project src/ImuViewer.App/ImuViewer.App.csproj
```

`AF_BLUETOOTH` socket を直接開くため、Linux では実行ファイルに `cap_net_raw` が
必要 (または `sudo`)。

```bash
sudo setcap cap_net_raw,cap_net_admin+ep "$(realpath "$(which dotnet)")"
```

## 関連ドキュメント

- フレーム/コマンド仕様: `docs/{en,ja}/development/pc-receive-spp.md`
- Hub 側コマンドハンドラ: `apps/btsensor/btsensor_cmd.c`
