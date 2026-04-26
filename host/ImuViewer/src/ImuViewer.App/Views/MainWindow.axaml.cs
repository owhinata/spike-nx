using Avalonia.Controls;
using ImuViewer.App.ViewModels;

namespace ImuViewer.App.Views;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();
        DataContext = new MainViewModel();
    }
}
