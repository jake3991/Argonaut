# bluerov_launch

## Installation

### [rqt_multiplot_plugin](https://github.com/ANYbotics/rqt_multiplot_plugin)

- Install [variant](https://github.com/ANYbotics/variant). Check out [this](https://github.com/ANYbotics/variant/pull/7) pull request if error occurs.

- Set `QWT_LIBRARIES` to `rqt_multiplot_plugin/rqt_multiplot/CMakeLists.txt` if not found.

    ```
    set(QWT_LIBRARIES /usr/lib/libqwt-qt5.so.6)
    ```


