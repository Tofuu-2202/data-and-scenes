#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <cstdlib> 
#define M_PI 3.14159265358979323846
using namespace std;
using namespace std::filesystem;
struct Point3D {
    double x, y, z;
};
double dotProduct(Point3D v1, Point3D v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
double magnitude(Point3D v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
Point3D calculateUnitVector(Point3D start, Point3D end) {
    double vec_x = end.x - start.x;
    double vec_y = end.y - start.y;
    double vec_z = end.z - start.z;
    double mag = magnitude({vec_x, vec_y, vec_z});
    return {vec_x / mag, vec_y / mag, vec_z / mag};
}
double calculateAngleBetweenVectors(Point3D P1, Point3D P2, Point3D P3) {
    Point3D P2P1 = calculateUnitVector(P2, P1);
    Point3D P2P3 = calculateUnitVector(P2, P3);
    double dot = dotProduct(P2P1, P2P3);
    dot = max(-1.0, min(1.0, dot));
    double angle = acos(dot);
    return angle;
}
Point3D calculateBisectorVector(Point3D P1, Point3D P2, Point3D P3) {
    Point3D P2P1 = calculateUnitVector(P2, P1);
    Point3D P2P3 = calculateUnitVector(P2, P3);
    Point3D bisector = {P2P1.x + P2P3.x, P2P1.y + P2P3.y, P2P1.z + P2P3.z};
    double magnitude1 = magnitude(bisector);
    return {bisector.x / magnitude1, bisector.y / magnitude1, bisector.z / magnitude1};
}
Point3D calculateFootOfPerpendicular(Point3D O, Point3D P1, Point3D P2) {
    Point3D P1P2 = calculateUnitVector(P1, P2);
    double t = ((O.x - P1.x) * P1P2.x + (O.y - P1.y) * P1P2.y + (O.z - P1.z) * P1P2.z) /
               (P1P2.x * P1P2.x + P1P2.y * P1P2.y + P1P2.z * P1P2.z);
    return {P1.x + t * P1P2.x, P1.y + t * P1P2.y, P1.z + t * P1P2.z};
}
Point3D crossProduct(Point3D v1, Point3D v2) {
    return {v1.y * v2.z - v1.z * v2.y, 
            v1.z * v2.x - v1.x * v2.z, 
            v1.x * v2.y - v1.y * v2.x};
}
double calculateVectorLength(Point3D A, Point3D B) {
    return sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2) + pow(B.z - A.z, 2));
}
array<Point3D, 4> createRectangleWithTwoPoints(Point3D center, double a, double b, Point3D direction, Point3D vec1, Point3D vec2) {
    array<Point3D, 4> vertices;
    vertices[0] = {center.x + a / 2 * vec1.x + b / 2 * vec2.x,
                  center.y + a / 2 * vec1.y + b / 2 * vec2.y,
                  center.z + a / 2 * vec1.z + b / 2 * vec2.z};
    vertices[1] = {center.x - a / 2 * vec1.x + b / 2 * vec2.x,
                  center.y - a / 2 * vec1.y + b / 2 * vec2.y,
                  center.z - a / 2 * vec1.z + b / 2 * vec2.z};
    vertices[2] = {center.x - a / 2 * vec1.x - b / 2 * vec2.x,
                  center.y - a / 2 * vec1.y - b / 2 * vec2.y,
                  center.z - a / 2 * vec1.z - b / 2 * vec2.z};
    vertices [3] = {center.x + a / 2 * vec1.x - b / 2 * vec2.x,
                  center.y + a / 2 * vec1.y - b / 2 * vec2.y,
                  center.z + a / 2 * vec1.z - b / 2 * vec2.z};
    return vertices;
}
Point3D calculateMidArcPoint(Point3D start, Point3D end) {
    return {(start.x + end.x) / 2, (start.y + end.y) / 2, (start.z + end.z) / 2};
}
Point3D calculateArcPoint(Point3D midArc, Point3D origin, double t) {
    double vec_x = midArc.x - origin.x;
    double vec_y = midArc.y - origin.y;
    double vec_z = midArc.z - origin.z;
    double magnitude = sqrt(vec_x * vec_x + vec_y * vec_y + vec_z * vec_z);
    double unit_x = vec_x / magnitude;
    double unit_y = vec_y / magnitude;
    double unit_z = vec_z / magnitude;
    Point3D arcPoint;
    arcPoint.x = midArc.x + unit_x * t;
    arcPoint.y = midArc.y + unit_y * t;
    arcPoint.z = midArc.z + unit_z * t;
    return arcPoint;
}
Point3D calculateArcPointImproved(Point3D midArc, Point3D O, double r, double b, double angle, bool useR) {
    double cosTerm = cos((M_PI - angle) / 2);
    double t = useR ? (r - r * cosTerm) : ((r + b) - (r + b) * cosTerm);
    return calculateArcPoint(midArc, O, t);
}
void processArcPoints(Point3D O1, Point3D O2, array<Point3D, 4>& midArcs, double r, double b, double angle, array<Point3D, 4>& arcPoints) {
    for (size_t i = 0; i < midArcs.size(); i++) {
        if (midArcs[i].x > 0) {
            for (size_t j = i + 1; j < midArcs.size(); j++) {
                if (midArcs[j].x > 0) {
                    double lenO1MidArcI = calculateVectorLength(O1, midArcs[i]);
                    double lenO1MidArcJ = calculateVectorLength(O1, midArcs[j]);
                    arcPoints[i] = calculateArcPointImproved(midArcs[i], O1, r, b, angle, lenO1MidArcI < lenO1MidArcJ);
                    arcPoints[j] = calculateArcPointImproved(midArcs[j], O1, r, b, angle, lenO1MidArcI > lenO1MidArcJ);
                    break;
                }
            }
        } else if (midArcs[i].x < 0) {
            for (size_t j = i + 1; j < midArcs.size(); j++) {
                if (midArcs[j].x < 0) {
                    double lenO2MidArcI = calculateVectorLength(O2, midArcs[i]);
                    double lenO2MidArcJ = calculateVectorLength(O2, midArcs[j]);
                    arcPoints[i] = calculateArcPointImproved(midArcs[i], O2, r, b, angle, lenO2MidArcI < lenO2MidArcJ);
                    arcPoints[j] = calculateArcPointImproved(midArcs[j], O2, r, b, angle, lenO2MidArcI > lenO2MidArcJ);
                    break;
                }
            }
        }
    }
}
void reorderVertices(array<Point3D, 4>& currentFace, const array<Point3D, 4>& referenceFace) {
    vector<pair<double, pair<int, int>>> distances;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double distance = calculateVectorLength(referenceFace[i], currentFace[j]);
            distances.push_back({distance, {i, j}});
        }
    }
    sort(distances.begin(), distances.end());
    vector<int> assignedIndices(4, -1);
    vector<bool> assignedReference(4, false);
    vector<bool> assignedCurrent(4, false);
    for (const auto& entry : distances) {
        int refIndex = entry.second.first;
        int curIndex = entry.second.second;
        if (!assignedReference[refIndex] && !assignedCurrent[curIndex]) {
            assignedIndices[refIndex] = curIndex;
            assignedReference[refIndex] = true;
            assignedCurrent[curIndex] = true;
        }
    }
    array<Point3D, 4> reorderedFace;
    for (int i = 0; i < 4; ++i) {
        reorderedFace[i] = currentFace[assignedIndices[i]];
    }
    currentFace = reorderedFace;
}
void reorderAllFaces(vector<array<Point3D, 4>>& faces) {
    for (size_t i = 1; i < faces.size(); ++i) {
        reorderVertices(faces[i], faces[i - 1]);
    }
}
void writeBlockMeshDict(ofstream &meshFile, const vector<array<Point3D, 4>> &rectangles, const vector<Point3D> &arcPoints, int n) {
    meshFile << "FoamFile\n{\n    format      ascii;\n    class       dictionary;\n    object      blockMeshDict;\n}\n";
    meshFile << "convertToMeters 1.0;\n";
    meshFile << "vertices\n(\n";
    int vertexIndex = 0;
      for (const auto& rectangle : rectangles) {
        for (const auto& vertex : rectangle) {
            meshFile << "    (" << vertex.x << " " << vertex.y << " " << vertex.z << ")\n";
        }
    }
    meshFile << ");\n\nblocks\n(\n";
    int blockCounter = 1;
    for (int i = 0; i < 2 * n - 3; i++) {
        int vertexIndex = i * 4; 
        if (blockCounter % 2 == 0) {
            meshFile << "hex (" 
                    << vertexIndex << " " << vertexIndex+1 << " " << vertexIndex+2 << " " << vertexIndex+3 << " " 
                    << vertexIndex+4 << " " << vertexIndex+5 << " " << vertexIndex+6 << " " << vertexIndex+7 
                    << ") (10 10 10) simpleGrading (1 1 1)\n";
        } 
        else {
            meshFile << "hex (" 
                    << vertexIndex << " " << vertexIndex+1 << " " << vertexIndex+2 << " " << vertexIndex+3 << " " 
                    << vertexIndex+4 << " " << vertexIndex+5 << " " << vertexIndex+6 << " " << vertexIndex+7 
                    << ") (10 10 1) simpleGrading (1 1 1)\n";
        }
        vertexIndex += 4;
        blockCounter++;
    }
meshFile << ");\n\n";
meshFile << "edges\n(\n";
int arcIndex = 8; 
for (int i = 0; i < arcPoints.size(); i += 4) {
    meshFile << "arc " << arcIndex << " " << (arcIndex - 4) 
            << "(" << arcPoints[i].x << " " << arcPoints[i].y << " " << arcPoints[i].z << ")\n";
    meshFile << "arc " << (arcIndex + 1) << " " << (arcIndex - 3) 
            << "(" << arcPoints[i+2].x << " " << arcPoints[i+2].y << " " << arcPoints[i+2].z << ")\n";
    meshFile << "arc " << (arcIndex + 2) << " " << (arcIndex - 2)
            << "(" << arcPoints[i+3].x << " " << arcPoints[i+3].y << " " << arcPoints[i+3].z << ")\n";
    meshFile << "arc " << (arcIndex + 3) << " " << (arcIndex - 1)
            << "(" << arcPoints[i+1].x << " " << arcPoints[i+1].y << " " << arcPoints[i+1].z << ")\n";
    arcIndex += 8;
}
meshFile << ");\n";
    meshFile << "boundary\n(\n";
    meshFile << "inlet\n{\n type patch;\n faces\n(\n (" 
            << 0 << " " << 3 << " " << 2 << " " << 1 << ")\n );\n}\n";
    int lastRectangleStart = (n - 1) * 8 - 4; 
    meshFile << "outlet\n{\n type patch;\n faces\n(\n (" 
            << lastRectangleStart << " " << lastRectangleStart+1 << " " << lastRectangleStart+2 << " " << lastRectangleStart+3 
            << ")\n );\n}\n";
    meshFile << "walls\n{\n type wall;\n faces\n(\n";
    for (int i = 0; i < n - 1; i++) {
        int base = i * 8;
        meshFile << "(" << base << " " << base+3 << " " << base+7 << " " << base+4 << ")\n";
        meshFile << "(" << base+1 << " " << base+5 << " " << base+6 << " " << base+2 << ")\n";
        meshFile << "(" << base << " " << base+4 << " " << base+5 << " " << base+1 << ")\n";
        meshFile << "(" << base+2 << " " << base+6 << " " << base+7 << " " << base+3 << ")\n";
          if (i < n - 2) { 
        int nextBase = base + 8;
        meshFile << "(" << base+4 << " " << base+7 << " " << nextBase+3 << " " << nextBase << ")\n";
        meshFile << "(" << base+5 << " " << base+9 << " " << nextBase+2 << " " << base+6 << ")\n";
        meshFile << "(" << base+4 << " " << nextBase << " " << nextBase+1 << " " << base+5 << ")\n";
        meshFile << "(" << base+6 << " " << nextBase+2 << " " << nextBase+3 << " " << base+7 << ")\n";
    }
    }
    meshFile << ");\n}\n);\n";
}

// bool runOpenFOAMCommands(const std::string& casePath) {
//     // Chuyển đường dẫn Windows thành đường dẫn WSL
//     std::string wslCasePath = "cd" +casePath ;
//     std::cout << "wslCasePath: " << wslCasePath << endl ;

//     std::string blockMeshCommand = " blockMesh" ;
//     std::cout << "blockMeshCommand: " << blockMeshCommand << std::endl;
//     std::cout << "Đang chạy blockMesh..." << std::endl;

    
//     if (system(blockMeshCommand.c_str()) != 0) {
//         std::cerr << "Lỗi khi chạy blockMesh" << std::endl;
//         return false;
//     }

//     // Lệnh foamToVTK
//     std::string foamToVTKCommand= "foamToVTK" ;
//     std::cout << "foamToVTKCommand: " << foamToVTKCommand << std::endl;
//     std::cout << "Đang chạy foamToVTK..." << std::endl;
//     if (system(foamToVTKCommand.c_str()) != 0) {
//         std::cerr << "Lỗi khi chạy foamToVTK" << std::endl;
//         return false;
//     }
//     std::cout << "Tất cả lệnh đã chạy thành công!" << std::endl;
//     return true;
  
// }
bool runOpenFOAMCommands(const std::string& casePath) {
    // Chuyển vào thư mục casePath
    std::string changeDirCommand = "cd " + casePath;

    // Lệnh blockMesh
    std::string blockMeshCommand = changeDirCommand + " && blockMesh";
    std::cout << "Đang chạy blockMesh..." << std::endl;

    if (system(blockMeshCommand.c_str()) != 0) {
        std::cerr << "Lỗi khi chạy blockMesh" << std::endl;
        return false;
    }

    // Lệnh foamToVTK
    std::string foamToVTKCommand = changeDirCommand + " && foamToVTK";
    std::cout << "Đang chạy foamToVTK..." << std::endl;

    if (system(foamToVTKCommand.c_str()) != 0) {
        std::cerr << "Lỗi khi chạy foamToVTK" << std::endl;
        return false;
    }

    std::cout << "Tất cả lệnh đã chạy thành công!" << std::endl;
    return true;
}
int readJSON()
{
std::ifstream inputFile("output.json");
    if (!inputFile.is_open()) {
        std::cerr << "Lỗi: Không thể mở file JSON!" << std::endl;
        return 1;
    }

    // Parse dữ liệu từ file JSON
    json jsonData;
    inputFile >> jsonData;
    inputFile.close();

    // Lấy các giá trị r, a, b
    double r = jsonData["r"];
    double a = jsonData["a"];
    double b = jsonData["b"];

    std::cout << "Giá trị r: " << r << std::endl;
    std::cout << "Giá trị a: " << a << std::endl;
    std::cout << "Giá trị b: " << b << std::endl;

    // Lấy và in ra tọa độ xyz từ list_point
    std::cout << "Toạ độ các điểm (x, y, z):" << std::endl;
    for (const auto& point : jsonData["list_point"]) {
        double x = point["x"];
        double y = point["y"];
        double z = point["z"];
        std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
    }

    return 0;
}
int main() {
    try{
    // Tạo thư mục và file cần thiết
    create_directory("case");
    ofstream("case/Allrun").close();
    ofstream("case/Allclean").close();
    create_directory("case/0");
    ofstream("case/0/p").close();
    ofstream("case/0/T").close();
    ofstream("case/0/U").close();

    create_directory("case/constant");
    ofstream("case/constant/physicalProperties").close();
    ofstream("case/constant/g").close();
    ofstream("case/constant/momentumTransport").close();
    ofstream("case/constant/thermalphysicalTransport").close();

    create_directory("case/system");
    ofstream("case/system/blockMeshDict").close();
    ofstream("case/system/controlDict").close();
    ofstream("case/system/fvSchemes").close();
    ofstream("case/system/fvSolution").close();
    
    cout << "Cấu trúc thư mục và các file đã được tạo." << endl;
    int n;
    cout << "Nhập số lượng điểm: ";
    cin >> n;
    vector<Point3D> points(n);
    cout << "Nhập tọa độ các điểm (x, y, z):\n";
    for (int i = 0; i < n; i++) {
        cout << "Điểm P" << i + 1 << ": ";
        cin >> points[i].x >> points[i].y >> points[i].z;
    }
    double a, b, r;
    cout << "Nhập kích thước a, b và bán kính r:\n";
    cin >> a >> b >> r;
    vector<array<Point3D, 4>> rectangles;
    vector<Point3D> arcPoints;
    Point3D P1 = points[0];
    Point3D P2 = points[1];
    Point3D P3 = points[2];
    Point3D vectorP2P1 = calculateUnitVector(P2, P1);
    Point3D vectorP2P3 = calculateUnitVector(P2, P3);
    Point3D vectorP1P2 = calculateUnitVector(P1, P2);
    double dot = dotProduct(vectorP2P1, vectorP2P3);
    double angle = acos(dot);
    Point3D bisector = calculateBisectorVector(P1, P2, P3);
    Point3D O1 = {P2.x + bisector.x * ((r + b / 2) / cos(M_PI / 2 - angle / 2)),
                      P2.y + bisector.y * ((r + b / 2) / cos(M_PI / 2 - angle / 2)),
                      P2.z + bisector.z * ((r + b / 2) / cos(M_PI / 2 - angle / 2))};
    Point3D A1 = calculateFootOfPerpendicular(O1, P1, P2);
    Point3D vectorO1A1 = calculateUnitVector(O1, A1);
    Point3D perpP1P2_O1A1 = crossProduct(vectorO1A1, vectorP1P2);
    auto rectP1 = createRectangleWithTwoPoints(P1, a, b, vectorP1P2,perpP1P2_O1A1, vectorO1A1);
    rectangles.push_back(rectP1);
    vector<Point3D> A_points, B_points, O_points;
    for (int i = 1; i < n - 1; ++i) {
        Point3D P_prev = points[i - 1];
        Point3D P_curr = points[i];
        Point3D P_next = points[i + 1];
        double angle = calculateAngleBetweenVectors(P_prev, P_curr, P_next);
        Point3D bisector = calculateBisectorVector(P_prev, P_curr, P_next);
        double distance = (r + b / 2) / cos(M_PI / 2 - angle / 2);
        Point3D O_i = {P_curr.x + bisector.x * distance,
                       P_curr.y + bisector.y * distance,
                       P_curr.z + bisector.z * distance};
        Point3D Oi_1 = {O_i.x + a / 2, O_i.y, O_i.z};
        Point3D Oi_2 = {O_i.x - a / 2, O_i.y, O_i.z};             
        O_points.push_back(O_i);
        Point3D A_i = calculateFootOfPerpendicular(O_i, P_prev, P_curr);
        Point3D B_i = calculateFootOfPerpendicular(O_i, P_curr, P_next);
        A_points.push_back(A_i);
        B_points.push_back(B_i);
        Point3D vectorOiAi = calculateUnitVector(O_i, A_i);
        Point3D perpVectorAi = crossProduct(vectorOiAi, calculateUnitVector(P_prev, P_curr));
        auto rectAi = createRectangleWithTwoPoints(A_i, a, b, calculateUnitVector(P_prev,P_curr),perpVectorAi, vectorOiAi);
        rectangles.push_back(rectAi);
        Point3D vectorOiBi = calculateUnitVector(O_i, B_i);
        Point3D perpVectorBi = crossProduct(vectorOiBi, calculateUnitVector(P_curr, P_next));
        auto rectBi = createRectangleWithTwoPoints(B_i, a, b, calculateUnitVector(P_curr,P_next),perpVectorBi, vectorOiBi);
        rectangles.push_back(rectBi);
         array<Point3D, 4> midArcs = {
        calculateMidArcPoint(rectAi[0], rectBi[0]),
        calculateMidArcPoint(rectAi[3], rectBi[3]),
        calculateMidArcPoint(rectAi[1], rectBi[1]),
        calculateMidArcPoint(rectAi[2], rectBi[2])
    };
    array<Point3D, 4> localArcPoints; 
    processArcPoints(Oi_1, Oi_2, midArcs, r, b, angle, localArcPoints);
     for (const auto& point : localArcPoints) {
        arcPoints.push_back(point);
    }
    } 
    Point3D Pn_minus2 = points[n - 3];
    Point3D Pn_minus1 = points[n - 2];
    Point3D Pn = points[n - 1];
    Point3D vectorPn_1Pn_2 = calculateUnitVector(Pn_minus1, Pn_minus2);
    Point3D vectorPn_1Pn = calculateUnitVector(Pn_minus1, Pn);
    double dotn = dotProduct(vectorPn_1Pn_2, vectorPn_1Pn);
    double anglen = acos(dotn);
    Point3D bisector_n = calculateBisectorVector(Pn_minus2, Pn_minus1, Pn);
    double distance_n = (r + b / 2) / cos(M_PI / 2 - anglen / 2);
    Point3D On = {Pn_minus1.x + bisector_n.x * distance_n,
                  Pn_minus1.y + bisector_n.y * distance_n,
                  Pn_minus1.z + bisector_n.z * distance_n};
    Point3D Bn = calculateFootOfPerpendicular(On, Pn_minus1, Pn);
    Point3D vectorPn_1Pn_n = calculateUnitVector(Pn_minus1, Pn);
    Point3D vectorOnBn = calculateUnitVector(On, Bn);
    Point3D perpVectorPn = crossProduct(vectorOnBn, vectorPn_1Pn_n);
    auto rectPn = createRectangleWithTwoPoints(Pn, a, b, vectorPn_1Pn_n,perpVectorPn, vectorOnBn);
    rectangles.push_back(rectPn);
    reorderAllFaces(rectangles);
    ofstream pfile ("case/0/p", ios::binary);
    if (pfile.is_open()){
        pfile << "FoamFile\n{\n format ascii;\n";
        pfile << "class volScalarField;\n";
        pfile << "object p;\n}\n";
        pfile << "dimensions [ 0 2 -2 0 0 0 0 ];\n";
        pfile << "internalField uniform 0;\n";
        pfile << "boundaryField\n{\n";
        pfile << "inlet\n{\ntype zeroGradient;\n}\n";
        pfile << "outlet\n{\ntype zeroGradient;\n}\n";
        pfile << "walls\n{\ntype zeroGradient;\n}\n}";
        pfile.close();
    }
    ofstream Ufile("case/0/U", ios::binary);
    if (Ufile.is_open()){
        Ufile << "FoamFile\n{\n format ascii;\n";
        Ufile << "class volVectorField;\n";
        Ufile << "object U;\n}\n ";
        Ufile << "dimensions [0 1 -1 0 0 0 0];\n";
        Ufile << "internalField uniform (0 0 0);\n";
        Ufile << "boundaryField\n{\n";
        Ufile << "    inlet\n";
        Ufile << "    {\n";
        Ufile << "        type            fixedValue;\n";
        Ufile << "        value           uniform (0 0 10);\n";  // Tốc độ không khí tại inlet
        Ufile << "    }\n";
        Ufile << "    outlet\n";
        Ufile << "    {\n";
        Ufile << "        type            zeroGradient;\n";
        Ufile << "    }\n";
        Ufile << "    walls\n";
        Ufile << "    {\n";
        Ufile << "        type            noSlip;\n";
        Ufile << "    }\n";
        Ufile << "}\n";
        Ufile.close();
    }
    ofstream Tfile("case/0/T",ios::binary);
    if(Tfile.is_open()){
        Tfile << "FoamFile\n{\n";
        Tfile << "format ascii;\n";
        Tfile << "class volScalarField;\n";
        Tfile << "object T;\n}\n";
        Tfile << "dimensions [0 0 0 1 0 0 0];\n";
        Tfile << "internalField uniform 293;\n";
        Tfile << "boundaryField\n{\n";
        Tfile << "inlet\n{\n";
        Tfile << "type    fixedValue;\nvalue     uniform 293;\n}\n";
        Tfile << "outlet\n{\n";
        Tfile << "type zeroGradient;\n}\n";
        Tfile << "walls\n{\n";
        Tfile << "type zeroGradient;\n}\n}\n";
        Tfile.close();
    }
    ofstream gfile("case/constant/g", ios::binary);
    if(gfile.is_open()){
        gfile << "FoamFile\n{\n";
        gfile << "format ascii;\n";
        gfile << "class uniformDimensionedVectorField;\n";
        gfile << "object g;\n}\n";
        gfile << "dimensions [0 1 -2 0 0 0 0];\n";
        gfile << "value (0 0 0);\n";
        gfile.close();
    }
    ofstream momentumTransport("case/constant/momentumTransport", ios::binary);
    if(momentumTransport.is_open()){
        momentumTransport << "FoamFile\n{\n";
        momentumTransport << "format ascii;\n";
        momentumTransport << "class dictionary;\n";
        momentumTransport << "object momentumTransport;\n}\n";
        momentumTransport << "simulationType RAS;\n";
        momentumTransport << "RAS\n{\n";
        momentumTransport << "model  kEpsilon;\n";
        momentumTransport << "turbulence on;\n";
        momentumTransport << "printCoeffs on;}\n";
        momentumTransport.close();
    }
    ofstream meshFile("case/system/blockMeshDict", ios::binary);
    if (meshFile.is_open()) {
        writeBlockMeshDict(meshFile, rectangles, arcPoints, n);
        meshFile.close();
    }
    }catch(const filesystem_error& e){
        cerr << "Lỗi: " << e.what() << endl;
    }
    ofstream controlDictFile("case/system/controlDict", ios::binary);
if (controlDictFile.is_open()) {
    controlDictFile << "/*--------------------------------*- C++ -*----------------------------------*\\\n";
    controlDictFile << "  =========                 |\n";
    controlDictFile << "  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox\n";
    controlDictFile << "   \\\\    /   O peration     | Website:  https://openfoam.org\n";
    controlDictFile << "    \\\\  /    A nd           | Version:  12\n";
    controlDictFile << "     \\\\/     M anipulation  |\n";
    controlDictFile << "\\*---------------------------------------------------------------------------*/\n";
    controlDictFile << "FoamFile\n{\n";
    controlDictFile << "    format      ascii;\n";
    controlDictFile << "    class       dictionary;\n";
    controlDictFile << "    object      controlDict;\n";
    controlDictFile << "}\n";
    controlDictFile << "// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n";
    controlDictFile << "application     foamRun;\n\n";
    controlDictFile << "solver          fluid;\n\n";
    controlDictFile << "startFrom       startTime;\n\n";
    controlDictFile << "startTime       0;\n\n";
    controlDictFile << "stopAt          endTime;\n\n";
    controlDictFile << "endTime         5000;\n\n";
    controlDictFile << "deltaT          1;\n\n";
    controlDictFile << "writeControl    timeStep;\n\n";
    controlDictFile << "writeInterval   50;\n\n";
    controlDictFile << "purgeWrite      0;\n\n";
    controlDictFile << "writeFormat     ascii;\n\n";
    controlDictFile << "writePrecision  10;\n\n";
    controlDictFile << "writeCompression off;\n\n";
    controlDictFile << "timeFormat      general;\n\n";
    controlDictFile << "timePrecision   6;\n\n";
    controlDictFile << "runTimeModifiable true;\n\n";
    controlDictFile << "// ************************************************************************* //\n";

    controlDictFile.close();
}
    
    std::string casePath = "/home/tofuu-2202/OpenFOAM/tofuu-2202-12/case";
    
    if (!runOpenFOAMCommands(casePath)) {
        std::cerr << "Chương trình kết thúc với lỗi." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
    return 0;
}



