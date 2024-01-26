<div align="center">
    <img alt="Crescendo 2024 Icon" src="https://www.firstinspires.org/sites/default/files/first-in-show/game-and-season/crescendo-logo.svg" width="200px" />
    <h3><strong>frc-2024</strong></h3>
    <h7>Robot code for FRC 2024 Crescendo</h7>
</div>
<br>

## Prerequisites

Make sure you have [Git](https://git-scm.com/downloads) installed, as well as some form of code editor. [Visual Studio Code](https://code.visualstudio.com/) is recommended, as well as the [Microsoft Extension Pack for Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack).

## Installation

1. Open a terminal and [`cd`](https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/cd) into the directory you want to clone the repository into.
2. Install [Git](https://git-scm.com/downloads) if you haven't already.
3. Run the command `git clone https://github.com/Pigmice2733/frc-2024.git --recurse-submodules`.

And you're done! The project will have been downloaded into a new folder called `frc-2024`.

4. Optional: open the project and, if not already present, add a line to the top `.vscode/settings.json` that says `"java.settings.url": ".vscode/settings.prefs",`, and add a corresponding file `.vscode/settings.prefs` with two lines:
    org.eclipse.jdt.core.formatter.lineSplit = 80
    org.eclipse.jdt.core.formatter.comment.line_length = 80

## Building and Running

1. Open a terminal and [`cd`](https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/cd) into the project directory (`frc-2024`).
2. To build the code, run `./gradlew build`.
3. To run all unit tests, run `./gradlew test`.
4. To deploy the code to the robot, make sure you're connected to the robot either over WiFi or through Ethernet. Then run `./gradlew deploy`.