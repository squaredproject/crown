# Crown

This is all the code relating to the Crown burning man project of 2018.

## Crown

The Crown subdirectory is the Processing model for light control. Processing demands the name
of the directory is Crown, so Crown it is.

## model

Various design models

## Requirements
Java 8, 9, or 10
Gradle
Processing 3.0
git

## How to Run
You can either run the application through the Processing Application or run the app headlessly.  Use the Processing application when you want to use the UI and DJ patterns.  headless mode is only for when the crown app is running continuously on an odroid without a screen (showtime)

### Run Interactively with Processing
1.  Download Crown repo
    - git clone https://github.com/squaredproject/crown.git
2.  Open Processing 3.0
    - click File->Open
    - navigate to Crown directory
    - click on Crown.pde 
    - click on the arrow button to Run
3.  Once Processing has loaded the app, it should show you the Crown software and you can start making patterns

### Run Headlessly
1.  Download Crown repo from github
    - git clone https://github.com/squaredproject/crown.git
2.  Open terminal and navigate to repo directory
    - eg. cd ~/crown/Crown
3.  Compile java application using gradle
    - gradle build
4.  Excute shell script to run headlessly
    - ./run.sh
    - confirm script out says "LX Engine Started" at the end


