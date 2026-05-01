# Path Coverage ROS2 Navigation System

Welcome to the central repository documentation for the ROS2-based path coverage and autonomous navigation project.

To keep documentation clean, modular, and easy to navigate safely as the codebase scales, all primary details have been split into the dedicated `docs/` folder. Please refer to the specific topics below:

*   📖 [**Usage and Operations Guide**](./docs/usage.md)  
    How to quickly start the main system with `start_path_coverage.py` and instructions detailing the manual 3-Terminal execution procedure.

*   🚧 [**Testing and Tuning**](./docs/testing_and_tuning.md)  
    Explanation of the specific scripts used to validate trajectory paths (`test2`), measure 2D occupation grid completeness (`test3`), along with tuning tables detailing past issue logs and the `MinGroundHeight` SLAM floor resolution.

*   🛠️ [**Development and Architecture**](./docs/development.md)  
    A history log detailing modular iteration implementations (e.g., path subdivision mechanisms, map color semantic overlaying). It lists the core packages forming this codebase and references the foundational open-source codebases.
