using Avalonia.Controls;
using Avalonia.Markup.Xaml;

namespace ImuViewer.App.Views;

public partial class SensorPanelView : UserControl
{
    public SensorPanelView() => AvaloniaXamlLoader.Load(this);
}
