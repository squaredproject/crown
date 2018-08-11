rm *.class
cp ~/LP4J/lp4j-api/target/lp4j-api-1.0.jar .
cp ~/LP4J/lp4j-midi/target/lp4j-midi-1.0.jar .
javac  -cp ".:./lp4j-midi-1.0.jar:./lp4j-api-1.0.jar" HelloWorld.java

