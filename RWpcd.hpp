#pragma  once

/***************************************************************************
**	Copyright Daywei All rights reserved
**
**	Contents: A lightweight C++ library for reading and writing PCD files
**
**
***************************************************************************/
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <array>
#include <type_traits>

namespace cw
{
	/*!
	template 3D point class.

	The class defines a point in 3D space. Data type of the point coordinates is specified
	as a template parameter.
	*/
	template<class Type> class Point3             // pixel position in the screen space
	{
	public:
		Point3() :x(0), y(0), z(0)
		{
		};
		Point3(Type x0, Type y0, Type z0) :x(x0), y(y0), z(z0)
		{
		};
		~Point3(){};

	public:

		bool operator==(const Point3 &ToCompare) const
		{
			return (x == ToCompare.x) && (y == ToCompare.y) && (z == ToCompare.z);
		}

		bool operator!=(const Point3 &ToCompare) const
		{
			return (x != ToCompare.x) || (y != ToCompare.y) || (z != ToCompare.z);
		}// Point3 operator !=

		Point3<Type>& operator=(const Point3 &From)
		{
			if (this != &From)
			{
				x = From.x;
				y = From.y;
				z = From.z;
			}
			return *this;
		}// Point3 operator !=

	public: //make data member public for easy access.
		Type     x;
		Type     y;
		Type     z;
	}; //class Point3

	typedef Point3<short> Point3s;
	typedef Point3<int> Point3i;
	typedef Point3<float> Point3f;
	typedef Point3<double> Point3d;
	typedef Point3i Point3D;

	template<class Type> class LiDARPoint :public Point3<Type>
	{
	public:	
		LiDARPoint() {}
		LiDARPoint(Type x0, Type y0, Type z0) :Point3<Type>(x0,y0,z0)
		{
		}
		Type reflectivity;
		Type timestamp;
	};

	typedef LiDARPoint<double> LiDARPointD;


	template<typename T>
	struct is_allowed_type
	{
		static constexpr bool value = std::is_same_v<T, Point3d> || std::is_same_v<T, Point3i> || std::is_same_v<T, LiDARPointD>;
	};

	template<class Type> class CPointCloud
	{
		static_assert(is_allowed_type<Type>::value, "only support Point3d,Point3i,LiDARPointD");
	public:
		std::vector<Type> points;
		std::vector<std::array< uint8_t, 3>> colors; //rgb

		bool empty() const
		{
			return points.empty();
		}

		bool HaveColor() const
		{
			return !colors.empty() && points.size()== colors.size();
		}
	};

	typedef CPointCloud<Point3d> PointCloud;

	class CRWpcd
	{
	public:
		enum class PCDFormat
		{
			ASCII,
			BINARY
		};
	public:
		static bool WritePointClouldToStream(std::ostream& out, const aw::PointCloud& pointclould)
		{
			if (!out.good()) return false;
			
			PCDFormat format_ = PCDFormat::BINARY;

			bool has_rgb = pointclould.HaveColor();
			size_t num_points = pointclould.points.size();

			// --- write PCD head ---
			out << "# .PCD v0.7 - Point Cloud Data file format\n";
			out << "VERSION 0.7\n";
			out << "FIELDS x y z";
			if (has_rgb) out << " rgb";
			out << "\n";

			out << "SIZE 4 4 4";
			if (has_rgb) out << " 4";
			out << "\n";

			out << "TYPE F F F";
			if (has_rgb) out << " F";  // rgb as float
			out << "\n";

			out << "COUNT 1 1 1";
			if (has_rgb) out << " 1";
			out << "\n";

			out << "WIDTH " << num_points << "\n";
			out << "HEIGHT 1\n";
			out << "VIEWPOINT 0 0 0 1 0 0 0\n";
			out << "POINTS " << num_points << "\n";
			out << "DATA " << (format_ == PCDFormat::ASCII ? "ascii" : "binary") << "\n";

			if (format_ == PCDFormat::ASCII) {
				for (size_t i = 0; i < num_points; ++i) {
					const auto& pt = pointclould.points[i];
					out << pt.x << " " << pt.y << " " << pt.z;
					if (has_rgb) {
						uint8_t r = pointclould.colors[i][0];
						uint8_t g = pointclould.colors[i][1];
						uint8_t b = pointclould.colors[i][2];
						uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
							(static_cast<uint32_t>(g) << 8) |
							(static_cast<uint32_t>(b));
						float rgb_float;
						std::memcpy(&rgb_float, &rgb, sizeof(uint32_t));
						out << " " << rgb_float;
					}
					out << "\n";
				}
			}
			else { // binary
				for (size_t i = 0; i < num_points; ++i) {
					float x = pointclould.points[i].x;
					float y = pointclould.points[i].y;
					float z = pointclould.points[i].z;
					out.write(reinterpret_cast<const char*>(&x), sizeof(float));
					out.write(reinterpret_cast<const char*>(&y), sizeof(float));
					out.write(reinterpret_cast<const char*>(&z), sizeof(float));

					if (has_rgb) {
						uint8_t r = pointclould.colors[i][0];
						uint8_t g = pointclould.colors[i][1];
						uint8_t b = pointclould.colors[i][2];
						uint8_t a = 255;
						uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
							(static_cast<uint32_t>(g) << 8) |
							(static_cast<uint32_t>(b)) |
							(static_cast<uint32_t>(a) << 24);
						float rgb_float;
						std::memcpy(&rgb_float, &rgb, sizeof(uint32_t));
						out.write(reinterpret_cast<const char*>(&rgb_float), sizeof(float));
					}
				}
			}

			return out.good();
		}

		static bool ReadPointClouldFromStream(std::istream& in, aw::PointCloud& pointclould)
		{
			if (!in.good()) return false;

			std::string line;
			bool data_ascii = false;
			bool has_rgb = false;
			size_t num_points = 0;

			// --- parse head ---
			while (std::getline(in, line)) {
				if (line.empty() || line[0] == '#') continue;

				std::istringstream iss(line);
				std::string header_key;
				iss >> header_key;

				if (header_key == "FIELDS") {
					std::string field;
					while (iss >> field) {
						if (field == "rgb") has_rgb = true;
					}
				}
				else if (header_key == "POINTS") {
					iss >> num_points;
				}
				else if (header_key == "DATA") {
					std::string data_type;
					iss >> data_type;
					data_ascii = (data_type == "ascii");
					break; // head end
				}
			}

			if (num_points == 0) return false;

			pointclould.points.resize(num_points);
			if (has_rgb) pointclould.colors.resize(num_points);

			// --- read data ---
			if (data_ascii)
			{
				for (size_t i = 0; i < num_points; ++i) {
					float x, y, z, rgb_f = 0.0f;
					if (has_rgb) {
						in >> x >> y >> z >> rgb_f;
					}
					else {
						in >> x >> y >> z;
					}
					pointclould.points[i].x = x;
					pointclould.points[i].y = y;
					pointclould.points[i].z = z;
					if (has_rgb) {
						uint32_t rgb;
						std::memcpy(&rgb, &rgb_f, sizeof(uint32_t));
						uint8_t r = (rgb >> 16) & 0x0000ff;
						uint8_t g = (rgb >> 8) & 0x0000ff;
						uint8_t b = (rgb >> 0) & 0x0000ff;
						pointclould.colors[i][0] = r;
						pointclould.colors[i][1] = g;
						pointclould.colors[i][2] = b;
					}
				}
			}
			else 
			{ // binary
				for (size_t i = 0; i < num_points; ++i)
				{
					float x, y, z;
					in.read(reinterpret_cast<char*>(&x), sizeof(float));
					in.read(reinterpret_cast<char*>(&y), sizeof(float));
					in.read(reinterpret_cast<char*>(&z), sizeof(float));
					pointclould.points[i].x = x;
					pointclould.points[i].y = y;
					pointclould.points[i].z = z;

					if (has_rgb) {
						float rgb_f;
						in.read(reinterpret_cast<char*>(&rgb_f), sizeof(float));
						uint32_t rgb;
						std::memcpy(&rgb, &rgb_f, sizeof(uint32_t));
						uint8_t r = (rgb >> 16) & 0x0000ff;
						uint8_t g = (rgb >> 8) & 0x0000ff;
						uint8_t b = (rgb >> 0) & 0x0000ff;
						pointclould.colors[i][0] = r;
						pointclould.colors[i][1] = g;
						pointclould.colors[i][2] = b;
					}
				};

			}
			return true;
		}

		static bool WritePointClouldToFile(const std::string& outFile, const aw::PointCloud& pointclould)
		{
			std::ofstream of(outFile, std::ios::binary);
			if (of.is_open())
			{
				return WritePointClouldToStream(of, pointclould);
			}
			return false;
		}

		static bool ReadPointClouldFromStream(const std::string& inFile, aw::PointCloud& pointclould)
		{
			std::ifstream ifs(inFile, std::ios::binary);
			if (ifs.is_open())
			{
				return ReadPointClouldFromStream(ifs, pointclould);
			}

			return false;
		}
	};
}


