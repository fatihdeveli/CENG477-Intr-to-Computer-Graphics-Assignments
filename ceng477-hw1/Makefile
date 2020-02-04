all: main.o Camera.o Image.o Light.o Material.o Ray.o Scene.o Shape.o tinyxml2.o VectorOperations.o
	g++ main.o Camera.o Image.o Light.o Material.o Ray.o Scene.o Shape.o tinyxml2.o VectorOperations.o -std=c++11 -O3 -o raytracer

main.o: main.cpp
	g++ -c main.cpp

Camera.o: Camera.cpp Camera.h
	g++ -c Camera.cpp

Image.o: Image.cpp Image.h
	g++ -c Image.cpp

Light.o: Light.cpp Light.h
	g++ -c Light.cpp

Material.o: Material.cpp Material.h
	g++ -c Material.cpp

Ray.o: Ray.cpp Ray.h
	g++ -c Ray.cpp

Scene.o: Scene.cpp Scene.h
	g++ -c Scene.cpp

Shape.o: Shape.cpp Shape.h
	g++ -c Shape.cpp

tinyxml2.o: tinyxml2.cpp tinyxml2.h
	g++ -c tinyxml2.cpp

VectorOperations.o: VectorOperations.cpp VectorOperations.h
	g++ -c VectorOperations.cpp

run1:
	make
	./raytracer inputs/input01.xml

run2:
	make
	./raytracer inputs/input02.xml

run3:
	make
	./raytracer inputs/input03.xml

run4:
	make
	./raytracer inputs/input04.xml

run5:
	make
	./raytracer inputs/input05.xml

clean:
	rm *.o raytracer
