#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <initializer_list>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include "Matrix.hpp"
#include "Vec.hpp"
#include "Point.hpp"
#include "BitmapImage.hpp"


// #define debug(x) { std::cout << #x << " = " << (x) << std::endl; }
# define debug(x) {}


using Matrix_4x4 = Matrix<double, 4, 4>;
using Matrix_4x1 = Matrix<double, 4, 1>;
using Vec3D = Vec<double, 3>;
using Point3D = Point<double>;


const double ESP = 1e-10;

struct RGB {
    unsigned r, g, b;
};

// Apply transformation matrix to a point
Point3D transformPoint(
    const Point3D& point, 
    const Matrix_4x4& transformationMatrix
) {
    auto pointMatrix = point.toMatrix();
    auto transformedPointMatrix = transformationMatrix * pointMatrix;

    return Point3D{
        transformedPointMatrix[0][0] / transformedPointMatrix[3][0],
        transformedPointMatrix[1][0] / transformedPointMatrix[3][0],
        transformedPointMatrix[2][0] / transformedPointMatrix[3][0]
    };
}


// Apply translation to the transformation matrix
Matrix_4x4 performTranslation(
    const Matrix_4x4& transformationMatrix,
    double tx, double ty, double tz
) {
    Matrix_4x4 translationMatrix{
        1, 0, 0, tx,
        0, 1, 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1
    };

    return transformationMatrix * translationMatrix;
}


// Apply scaling to the transformation matrix
Matrix_4x4 performScaling(
    const Matrix_4x4& transformationMatrix,
    double sx, double sy, double sz
) {
    Matrix_4x4 scalingMatrix{
        sx, 0, 0, 0,
        0, sy, 0, 0,
        0, 0, sz, 0,
        0, 0, 0, 1
    };

    return transformationMatrix * scalingMatrix;
}


inline double degToRad(double deg) {
    const double PI = acos(-1);
    return deg * PI / 180;
}


// Rotate vec along axis by angleDeg
Vec3D rotateVector(
    const Vec3D& vec,
    const Vec3D& axis,
    double angleDeg    
) {
    double angleRad = degToRad(angleDeg);
    auto t1 = cos(angleRad) * vec;
    auto t2 = ((1 - cos(angleRad)) * axis.dot(vec)) * axis;
    auto t3 = sin(angleRad) * axis.cross(vec);

    return t1 + t2 + t3;
}


// Apply rotation to the transformation matrix
Matrix_4x4 performRotation(
    const Matrix_4x4& transformationMatrix,
    double angleDeg, double ax, double ay, double az
) {
    Vec3D i{1, 0, 0};
    Vec3D j{0, 1, 0};
    Vec3D k{0, 0, 1};
    Vec3D a{ax, ay, az};
    a.normalize();
    
    auto c1 = rotateVector(i, a, angleDeg);
    auto c2 = rotateVector(j, a, angleDeg);
    auto c3 = rotateVector(k, a, angleDeg);

    Matrix_4x4 rotationMatrix{
        c1[0], c2[0], c3[0], 0,
        c1[1], c2[1], c3[1], 0,
        c1[2], c2[2], c3[2], 0,
        0, 0, 0, 1
    };

    return transformationMatrix * rotationMatrix;
}


// Perform modeling transform
std::vector<std::array<Point3D, 3>>
modelingTransform(
    std::ifstream& inputFile,
    const std::string& outputFilename    
) {
    // Start with an identity transformation matrix
    auto transformationMatrix = Matrix_4x4::identity();
    // Stack for transformation matices
    std::vector<Matrix_4x4> transformationMatrixStack;
    // Generated scene i.e. triangles
    std::vector<std::array<Point3D, 3>> scene;

    while (true) {
        std::string command;
        inputFile >> command;

        if (command == "triangle") {
            std::array<Point<double>, 3> triangle;
            for (size_t i = 0; i < 3; ++i) {
                inputFile >> triangle[i];
            }

            std::array<Point<double>, 3> transformedTriangle;
            for (size_t i = 0; i < 3; ++i) {
                transformedTriangle[i] = transformPoint(triangle[i], transformationMatrix);
            }

            scene.push_back(transformedTriangle);
        }
        else if (command == "translate") {
            double tx, ty, tz;
            inputFile >> tx >> ty >> tz;

            transformationMatrix = performTranslation(transformationMatrix, tx, ty, tz);
        }
        else if (command == "scale") {
            double sx, sy, sz;
            inputFile >> sx >> sy >> sz;

            transformationMatrix = performScaling(transformationMatrix, sx, sy, sz);
        }
        else if (command == "rotate") {
            double angle, ax, ay, az;
            inputFile >> angle >> ax >> ay >> az;

            transformationMatrix = performRotation(transformationMatrix, angle, ax, ay, az);
        }
        else if (command == "push") {
            transformationMatrixStack.push_back(transformationMatrix);
        }
        else if (command == "pop") {
            if (!transformationMatrixStack.empty()) {
                transformationMatrix = transformationMatrixStack.back();
                transformationMatrixStack.pop_back();
            }
            else {
                throw std::out_of_range("Error: Pop operation cannot be performed. Stack is empty.");
            }
        }
        else if (command == "end") {
            break;
        }
    }

    std::ofstream outputFile(outputFilename);
    if (!outputFile.is_open()) {
        throw std::runtime_error("Cannot open file " + outputFilename);
    }

    for (const auto& triangle : scene) {
        for (const auto& point : triangle) {
            outputFile << point << '\n';
        }
        outputFile << '\n';
    }

    outputFile.close();

    return scene;
}


std::vector<std::array<Point3D, 3>> 
viewTransform(
    const Point3D& eye,
    const Point3D& look,
    const Vec3D& up,
    const std::vector<std::array<Point3D, 3>>& scene,
    const std::string& outputFilename 
) {
    auto l = look - eye;
    l.normalize();
    auto r = l.cross(up);
    r.normalize();
    auto u = r.cross(l);

    Matrix_4x4 translationMatrix{
        1, 0, 0, -eye.x,
        0, 1, 0, -eye.y,
        0, 0, 1, -eye.z,
        0, 0, 0, 1
    };
    Matrix_4x4 rotationMatrix{
        r[0], r[1], r[2], 0,
        u[0], u[1], u[2], 0,
        -l[0], -l[1], -l[2], 0,
        0, 0, 0, 1
    };
    auto viewTransformationMatrix = rotationMatrix * translationMatrix;

    std::vector<std::array<Point3D, 3>> newScene;
    for (auto triangle : scene) {
        for (auto& point : triangle) {
            point = transformPoint(point, viewTransformationMatrix);
        }
        newScene.push_back(triangle);
    }

    std::ofstream outputFile(outputFilename);
    if (!outputFile.is_open()) {
        throw std::runtime_error("Cannot open file " + outputFilename);
    }

    for (const auto& triangle : newScene) {
        for (const auto& point : triangle) {
            outputFile << point << '\n';
        }
        outputFile << '\n';
    }

    outputFile.close();

    return newScene;
}


std::vector<std::array<Point3D, 3>> 
projectionTransform(
    double fovY,
    double aspectRatio,
    double near,
    double far,
    const std::vector<std::array<Point3D, 3>>& scene,
    const std::string& outputFilename
) {
    double fovX = fovY * aspectRatio;
    double t = near * tan(degToRad(fovY/2)); 
    double r = near * tan(degToRad(fovX/2));

    Matrix_4x4 projectionMatrix{
        near / r, 0, 0, 0,
        0, near / t, 0, 0,
        0, 0, - (far + near) / (far - near), - (2 * far * near) / (far - near),
        0, 0, -1, 0
    };

    std::vector<std::array<Point3D, 3>> newScene;
    for (auto triangle : scene) {
        for (auto& point : triangle) {
            point = transformPoint(point, projectionMatrix);
        }
        newScene.push_back(triangle);
    }
    
    std::ofstream outputFile(outputFilename);
    if (!outputFile.is_open()) {
        throw std::runtime_error("Cannot open file " + outputFilename);
    }

    for (const auto& triangle : newScene) {
        for (const auto& point : triangle) {
            outputFile << point << '\n';
        }
        outputFile << '\n';
    }

    outputFile.close();

    return newScene;
}


static unsigned long int g_seed = 1;
inline unsigned random() {
    g_seed = (214013 * g_seed + 2531011);
    return (g_seed >> 16) & 0x7FFF;
}


inline double isEqual(double a, double b) {
    return abs(a - b) <= ESP;
}


inline Point3D
getLinePointFromY(const Point3D& p1, const Point3D& p2, double y) {
    double x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
    double z = p1.z + (y - p1.y) * (p2.z - p1.z) / (p2.y - p1.y);
    return Point3D(x, y, z);
}


void rasterize(
    const std::vector<std::array<Point3D, 3>>& scene,
    unsigned screenWidth,
    unsigned screenHeight,
    const std::string& outputFilename,
    const std::string& imageFilename
) {
    double dx = 2.0 / screenWidth;
    double dy = 2.0 / screenHeight;
    double leftX = -1.0 + dx / 2;
    double rightX = 1.0 - dx / 2;
    double topY = 1.0 - dy / 2;
    double bottomY = -1.0 + dy / 2;

    auto image = bitmap_image(screenWidth, screenHeight);

    double maxZ = 1.0, minZ = -1.0;
    std::vector<std::vector<double>> zBuffer(
        screenHeight, 
        std::vector<double>(screenWidth, maxZ)
    );

    for (auto triangle : scene) {
        RGB color {random(), random(), random()};

        double minY = INFINITY, maxY = -INFINITY;
        for (auto point : triangle) {
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }

        if (minY > topY || maxY < bottomY) {
            continue;
        }
        minY = std::max(bottomY, minY);
        maxY = std::min(topY, maxY);

        unsigned topRow = round((topY - maxY) / dy);
        unsigned bottomRow = round((topY - minY) / dy);

        for (unsigned row = topRow; row <= bottomRow; ++row) {
            double currY = topY - row * dy;
            
            std::vector<Point3D> rowPoints;
            for (int i = 0; i < 2; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    const auto& p1 = triangle[i];
                    const auto& p2 = triangle[j];

                    double minY = std::min(p1.y, p2.y);
                    double maxY = std::max(p1.y, p2.y);

                    if (minY <= currY 
                        && currY <= maxY 
                        && !isEqual(minY, maxY)
                    ) {
                        rowPoints.push_back(
                            getLinePointFromY(p1, p2, currY)
                        );
                    }
                }
            }
            if (rowPoints.size() != 2) {
                continue;
            }

            if (rowPoints[0].x > rowPoints[1].x) {
                std::swap(rowPoints[0], rowPoints[1]);
            }
            auto leftPoint = rowPoints[0];
            auto rightPoint = rowPoints[1];

            if (rightPoint.x < leftX || leftPoint.x > rightX) {
                continue;
            }
            double minX = std::max(leftX, leftPoint.x);
            double maxX = std::min(rightX, rightPoint.x);

            unsigned leftCol = round((minX - leftX) / dx);
            unsigned rightCol = round((maxX - leftX) / dx);

            for (unsigned col = leftCol; col <= rightCol; ++col) {
                double currX = leftX + col * dx;

                auto const& p1 = leftPoint;
                auto const& p2 = rightPoint;
                double currZ = p1.z + (currX - p1.x) * (p2.z - p1.z) / (p2.x - p1.x);

                if (currZ > minZ && currZ < zBuffer[row][col]) {
                    zBuffer[row][col] = currZ;
                    image.set_pixel(col, row, color.r, color.g, color.b);
                }
            }
        }
    }

    image.save_image(imageFilename);

    std::ofstream outputFile(outputFilename);
    if (!outputFile.is_open()) {
        throw std::runtime_error("Cannot open file " + outputFilename);
    }

    for (unsigned row = 0; row < screenHeight; ++row) {
        for (unsigned col = 0; col < screenHeight; ++col) {
            if (zBuffer[row][col] < maxZ) {
                outputFile << std::fixed << std::setprecision(6)
                    << zBuffer[row][col] << "\t";
            }
        }
        outputFile << '\n';
    }

    outputFile.close();
}


int main(int argc, char* argv[]) {
    std::string inputFilename = "scene.txt";
    std::string configFilename = "config.txt";
    std::string outputFilename1 = "stage1.txt";
    std::string outputFilename2 = "stage2.txt";
    std::string outputFilename3 = "stage3.txt";
    std::string outputFilename4 = "z_buffer.txt";
    std::string imageFilename = "out.bmp";

    if (argc == 6) {
        outputFilename1 = argv[1];
        outputFilename2 = argv[2];
        outputFilename3 = argv[3];
        outputFilename4 = argv[4];
        imageFilename = argv[5];
    }

    std::ifstream sceneFile(inputFilename);
    if (!sceneFile.is_open()) {
        throw std::runtime_error("Cannot open file " + inputFilename);
    }
    std::ifstream configFile(configFilename);
    if (!sceneFile.is_open()) {
        throw std::runtime_error("Cannot open file " + configFilename);
    }

    double eyeX, eyeY, eyeZ;
    double lookX, lookY, lookZ;
    double upX, upY, upZ;
    double fovY, aspectRatio, near, far;
    int screenWidth, screenHeight;

    sceneFile >> eyeX >> eyeY >> eyeZ;
    sceneFile >> lookX >> lookY >> lookZ;
    sceneFile >> upX >> upY >> upZ;
    sceneFile >> fovY >> aspectRatio >> near >> far;
    
    configFile >> screenWidth >> screenHeight;

    auto scene = modelingTransform(sceneFile, outputFilename1);
    scene = viewTransform(
        Point3D{eyeX, eyeY, eyeZ},
        Point3D{lookX, lookY, lookZ},
        Vec3D{upX, upY, upZ},
        scene,
        outputFilename2
    );
    scene = projectionTransform(
        fovY,
        aspectRatio,
        near,
        far,
        scene,
        outputFilename3
    );
    rasterize(
        scene, 
        screenWidth,
        screenHeight,
        outputFilename4, 
        imageFilename
    );

    sceneFile.close();
    configFile.close();

    std::cout << "Rasterization successful!" << std::endl;

    return 0;
}
