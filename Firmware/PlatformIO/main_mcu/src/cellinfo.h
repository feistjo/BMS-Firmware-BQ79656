#pragma once
#include <map>

class CellInfo
{
public:
    virtual float VoltageToSOC(float voltage) = 0;
};

class INR21700P42A : public CellInfo
{
public:
    float VoltageToSOC(float voltage) override { return Interpolate(voltages_to_SOC_lut_, voltage); }

private:
    // consts from datasheet
    std::map<float, float> voltages_to_SOC_lut_{{4.186, 1},
                                                {4.177, 0.997619047619048},
                                                {4.168, 0.995238095238095},
                                                {4.159, 0.992857142857143},
                                                {4.149, 0.990476190476191},
                                                {4.142, 0.988095238095238},
                                                {4.136, 0.985714285714286},
                                                {4.131, 0.983333333333334},
                                                {4.126, 0.980952380952381},
                                                {4.121, 0.978571428571429},
                                                {4.116, 0.976190476190477},
                                                {4.113, 0.973809523809524},
                                                {4.109, 0.971428571428572},
                                                {4.106, 0.96904761904762},
                                                {4.103, 0.966666666666667},
                                                {4.099, 0.964285714285715},
                                                {4.097, 0.961904761904763},
                                                {4.095, 0.95952380952381},
                                                {4.094, 0.957142857142858},
                                                {4.092, 0.954761904761906},
                                                {4.09, 0.952380952380954},
                                                {4.088, 0.950000000000001},
                                                {4.087, 0.947619047619049},
                                                {4.085, 0.945238095238096},
                                                {4.083, 0.942857142857144},
                                                {4.082, 0.940476190476192},
                                                {4.08, 0.938095238095239},
                                                {4.079, 0.935714285714287},
                                                {4.077, 0.933333333333335},
                                                {4.075, 0.930952380952382},
                                                {4.074, 0.92857142857143},
                                                {4.072, 0.926190476190478},
                                                {4.071, 0.923809523809525},
                                                {4.069, 0.921428571428573},
                                                {4.068, 0.919047619047621},
                                                {4.066, 0.916666666666668},
                                                {4.064, 0.914285714285716},
                                                {4.063, 0.911904761904764},
                                                {4.061, 0.909523809523812},
                                                {4.06, 0.907142857142859},
                                                {4.059, 0.904761904761907},
                                                {4.059, 0.902380952380955},
                                                {4.059, 0.900000000000002},
                                                {4.059, 0.89761904761905},
                                                {4.059, 0.895238095238098},
                                                {4.059, 0.892857142857145},
                                                {4.059, 0.890476190476193},
                                                {4.059, 0.888095238095241},
                                                {4.059, 0.885714285714288},
                                                {4.059, 0.883333333333336},
                                                {4.059, 0.880952380952384},
                                                {4.059, 0.878571428571431},
                                                {4.059, 0.876190476190479},
                                                {4.059, 0.873809523809527},
                                                {4.059, 0.871428571428574},
                                                {4.059, 0.869047619047622},
                                                {4.059, 0.86666666666667},
                                                {4.059, 0.864285714285717},
                                                {4.059, 0.861904761904765},
                                                {4.059, 0.859523809523813},
                                                {4.059, 0.85714285714286},
                                                {4.058, 0.854761904761908},
                                                {4.056, 0.852380952380956},
                                                {4.055, 0.850000000000003},
                                                {4.053, 0.847619047619051},
                                                {4.052, 0.845238095238099},
                                                {4.05, 0.842857142857146},
                                                {4.049, 0.840476190476194},
                                                {4.047, 0.838095238095242},
                                                {4.046, 0.835714285714289},
                                                {4.045, 0.833333333333337},
                                                {4.043, 0.830952380952385},
                                                {4.042, 0.828571428571432},
                                                {4.04, 0.82619047619048},
                                                {4.038, 0.823809523809528},
                                                {4.036, 0.821428571428575},
                                                {4.034, 0.819047619047623},
                                                {4.032, 0.816666666666671},
                                                {4.03, 0.814285714285718},
                                                {4.028, 0.811904761904766},
                                                {4.025, 0.809523809523814},
                                                {4.022, 0.807142857142861},
                                                {4.019, 0.804761904761909},
                                                {4.016, 0.802380952380957},
                                                {4.012, 0.800000000000004},
                                                {4.009, 0.797619047619052},
                                                {4.006, 0.7952380952381},
                                                {4.002, 0.792857142857147},
                                                {3.999, 0.790476190476195},
                                                {3.996, 0.788095238095243},
                                                {3.993, 0.78571428571429},
                                                {3.989, 0.783333333333338},
                                                {3.986, 0.780952380952386},
                                                {3.983, 0.778571428571433},
                                                {3.979, 0.776190476190481},
                                                {3.976, 0.773809523809529},
                                                {3.973, 0.771428571428576},
                                                {3.97, 0.769047619047624},
                                                {3.966, 0.766666666666672},
                                                {3.963, 0.764285714285719},
                                                {3.96, 0.761904761904767},
                                                {3.956, 0.759523809523815},
                                                {3.954, 0.757142857142862},
                                                {3.951, 0.75476190476191},
                                                {3.949, 0.752380952380958},
                                                {3.947, 0.750000000000005},
                                                {3.945, 0.747619047619053},
                                                {3.943, 0.745238095238101},
                                                {3.941, 0.742857142857148},
                                                {3.939, 0.740476190476196},
                                                {3.936, 0.738095238095244},
                                                {3.934, 0.735714285714291},
                                                {3.932, 0.733333333333339},
                                                {3.93, 0.730952380952387},
                                                {3.928, 0.728571428571434},
                                                {3.926, 0.726190476190482},
                                                {3.924, 0.72380952380953},
                                                {3.922, 0.721428571428577},
                                                {3.919, 0.719047619047625},
                                                {3.917, 0.716666666666673},
                                                {3.915, 0.71428571428572},
                                                {3.913, 0.711904761904768},
                                                {3.911, 0.709523809523816},
                                                {3.908, 0.707142857142863},
                                                {3.906, 0.704761904761911},
                                                {3.904, 0.702380952380959},
                                                {3.901, 0.700000000000006},
                                                {3.899, 0.697619047619054},
                                                {3.897, 0.695238095238102},
                                                {3.894, 0.692857142857149},
                                                {3.892, 0.690476190476197},
                                                {3.89, 0.688095238095245},
                                                {3.887, 0.685714285714292},
                                                {3.885, 0.68333333333334},
                                                {3.883, 0.680952380952388},
                                                {3.88, 0.678571428571436},
                                                {3.878, 0.676190476190483},
                                                {3.876, 0.673809523809531},
                                                {3.874, 0.671428571428579},
                                                {3.871, 0.669047619047626},
                                                {3.869, 0.666666666666674},
                                                {3.867, 0.664285714285722},
                                                {3.864, 0.661904761904769},
                                                {3.862, 0.659523809523817},
                                                {3.86, 0.657142857142865},
                                                {3.857, 0.654761904761912},
                                                {3.855, 0.65238095238096},
                                                {3.853, 0.650000000000008},
                                                {3.85, 0.647619047619055},
                                                {3.848, 0.645238095238103},
                                                {3.846, 0.642857142857151},
                                                {3.843, 0.640476190476198},
                                                {3.841, 0.638095238095246},
                                                {3.839, 0.635714285714294},
                                                {3.836, 0.633333333333341},
                                                {3.834, 0.630952380952389},
                                                {3.832, 0.628571428571437},
                                                {3.83, 0.626190476190484},
                                                {3.827, 0.623809523809532},
                                                {3.825, 0.62142857142858},
                                                {3.823, 0.619047619047627},
                                                {3.82, 0.616666666666675},
                                                {3.818, 0.614285714285723},
                                                {3.816, 0.61190476190477},
                                                {3.814, 0.609523809523818},
                                                {3.811, 0.607142857142866},
                                                {3.809, 0.604761904761913},
                                                {3.807, 0.602380952380961},
                                                {3.805, 0.600000000000009},
                                                {3.802, 0.597619047619056},
                                                {3.8, 0.595238095238104},
                                                {3.798, 0.592857142857152},
                                                {3.796, 0.590476190476199},
                                                {3.793, 0.588095238095247},
                                                {3.791, 0.585714285714295},
                                                {3.789, 0.583333333333342},
                                                {3.787, 0.58095238095239},
                                                {3.784, 0.578571428571438},
                                                {3.782, 0.576190476190485},
                                                {3.78, 0.573809523809533},
                                                {3.778, 0.571428571428581},
                                                {3.775, 0.569047619047628},
                                                {3.773, 0.566666666666676},
                                                {3.771, 0.564285714285724},
                                                {3.769, 0.561904761904771},
                                                {3.766, 0.559523809523819},
                                                {3.764, 0.557142857142867},
                                                {3.762, 0.554761904761914},
                                                {3.76, 0.552380952380962},
                                                {3.757, 0.55000000000001},
                                                {3.755, 0.547619047619057},
                                                {3.753, 0.545238095238105},
                                                {3.751, 0.542857142857153},
                                                {3.748, 0.5404761904762},
                                                {3.746, 0.538095238095248},
                                                {3.744, 0.535714285714296},
                                                {3.742, 0.533333333333343},
                                                {3.739, 0.530952380952391},
                                                {3.737, 0.528571428571439},
                                                {3.735, 0.526190476190486},
                                                {3.733, 0.523809523809534},
                                                {3.73, 0.521428571428582},
                                                {3.728, 0.519047619047629},
                                                {3.726, 0.516666666666677},
                                                {3.723, 0.514285714285725},
                                                {3.721, 0.511904761904772},
                                                {3.719, 0.50952380952382},
                                                {3.717, 0.507142857142868},
                                                {3.714, 0.504761904761915},
                                                {3.712, 0.502380952380963},
                                                {3.71, 0.500000000000011},
                                                {3.707, 0.497619047619058},
                                                {3.705, 0.495238095238106},
                                                {3.703, 0.492857142857154},
                                                {3.7, 0.490476190476201},
                                                {3.698, 0.488095238095249},
                                                {3.696, 0.485714285714297},
                                                {3.694, 0.483333333333344},
                                                {3.691, 0.480952380952392},
                                                {3.689, 0.47857142857144},
                                                {3.687, 0.476190476190487},
                                                {3.684, 0.473809523809535},
                                                {3.682, 0.471428571428583},
                                                {3.68, 0.46904761904763},
                                                {3.678, 0.466666666666678},
                                                {3.675, 0.464285714285725},
                                                {3.673, 0.461904761904773},
                                                {3.671, 0.459523809523821},
                                                {3.668, 0.457142857142868},
                                                {3.666, 0.454761904761916},
                                                {3.664, 0.452380952380964},
                                                {3.661, 0.450000000000011},
                                                {3.659, 0.447619047619059},
                                                {3.657, 0.445238095238106},
                                                {3.655, 0.442857142857154},
                                                {3.652, 0.440476190476202},
                                                {3.65, 0.438095238095249},
                                                {3.648, 0.435714285714297},
                                                {3.645, 0.433333333333345},
                                                {3.643, 0.430952380952392},
                                                {3.641, 0.42857142857144},
                                                {3.639, 0.426190476190487},
                                                {3.637, 0.423809523809535},
                                                {3.636, 0.421428571428583},
                                                {3.634, 0.41904761904763},
                                                {3.633, 0.416666666666678},
                                                {3.631, 0.414285714285725},
                                                {3.629, 0.411904761904773},
                                                {3.628, 0.409523809523821},
                                                {3.626, 0.407142857142868},
                                                {3.624, 0.404761904761916},
                                                {3.623, 0.402380952380964},
                                                {3.621, 0.400000000000011},
                                                {3.619, 0.397619047619059},
                                                {3.618, 0.395238095238106},
                                                {3.616, 0.392857142857154},
                                                {3.615, 0.390476190476202},
                                                {3.613, 0.388095238095249},
                                                {3.611, 0.385714285714297},
                                                {3.61, 0.383333333333344},
                                                {3.608, 0.380952380952392},
                                                {3.606, 0.37857142857144},
                                                {3.605, 0.376190476190487},
                                                {3.603, 0.373809523809535},
                                                {3.601, 0.371428571428583},
                                                {3.6, 0.36904761904763},
                                                {3.598, 0.366666666666678},
                                                {3.597, 0.364285714285725},
                                                {3.595, 0.361904761904773},
                                                {3.593, 0.359523809523821},
                                                {3.592, 0.357142857142868},
                                                {3.59, 0.354761904761916},
                                                {3.588, 0.352380952380963},
                                                {3.587, 0.350000000000011},
                                                {3.585, 0.347619047619059},
                                                {3.583, 0.345238095238106},
                                                {3.582, 0.342857142857154},
                                                {3.58, 0.340476190476202},
                                                {3.578, 0.338095238095249},
                                                {3.577, 0.335714285714297},
                                                {3.575, 0.333333333333344},
                                                {3.573, 0.330952380952392},
                                                {3.57, 0.32857142857144},
                                                {3.568, 0.326190476190487},
                                                {3.565, 0.323809523809535},
                                                {3.563, 0.321428571428583},
                                                {3.561, 0.31904761904763},
                                                {3.558, 0.316666666666678},
                                                {3.556, 0.314285714285725},
                                                {3.553, 0.311904761904773},
                                                {3.551, 0.309523809523821},
                                                {3.548, 0.307142857142868},
                                                {3.546, 0.304761904761916},
                                                {3.543, 0.302380952380963},
                                                {3.541, 0.300000000000011},
                                                {3.538, 0.297619047619059},
                                                {3.536, 0.295238095238106},
                                                {3.533, 0.292857142857154},
                                                {3.531, 0.290476190476202},
                                                {3.529, 0.288095238095249},
                                                {3.526, 0.285714285714297},
                                                {3.524, 0.283333333333344},
                                                {3.521, 0.280952380952392},
                                                {3.519, 0.27857142857144},
                                                {3.516, 0.276190476190487},
                                                {3.514, 0.273809523809535},
                                                {3.511, 0.271428571428582},
                                                {3.509, 0.26904761904763},
                                                {3.506, 0.266666666666678},
                                                {3.504, 0.264285714285725},
                                                {3.502, 0.261904761904773},
                                                {3.499, 0.259523809523821},
                                                {3.497, 0.257142857142868},
                                                {3.494, 0.254761904761916},
                                                {3.492, 0.252380952380963},
                                                {3.489, 0.250000000000011},
                                                {3.487, 0.247619047619059},
                                                {3.484, 0.245238095238106},
                                                {3.482, 0.242857142857154},
                                                {3.479, 0.240476190476201},
                                                {3.477, 0.238095238095249},
                                                {3.475, 0.235714285714297},
                                                {3.473, 0.233333333333344},
                                                {3.471, 0.230952380952392},
                                                {3.468, 0.22857142857144},
                                                {3.466, 0.226190476190487},
                                                {3.464, 0.223809523809535},
                                                {3.462, 0.221428571428582},
                                                {3.46, 0.21904761904763},
                                                {3.458, 0.216666666666678},
                                                {3.456, 0.214285714285725},
                                                {3.454, 0.211904761904773},
                                                {3.451, 0.209523809523821},
                                                {3.449, 0.207142857142868},
                                                {3.447, 0.204761904761916},
                                                {3.445, 0.202380952380963},
                                                {3.443, 0.200000000000011},
                                                {3.441, 0.197619047619059},
                                                {3.439, 0.195238095238106},
                                                {3.437, 0.192857142857154},
                                                {3.434, 0.190476190476201},
                                                {3.432, 0.188095238095249},
                                                {3.43, 0.185714285714297},
                                                {3.428, 0.183333333333344},
                                                {3.426, 0.180952380952392},
                                                {3.424, 0.17857142857144},
                                                {3.422, 0.176190476190487},
                                                {3.419, 0.173809523809535},
                                                {3.417, 0.171428571428582},
                                                {3.415, 0.16904761904763},
                                                {3.413, 0.166666666666678},
                                                {3.411, 0.164285714285725},
                                                {3.409, 0.161904761904773},
                                                {3.407, 0.15952380952382},
                                                {3.405, 0.157142857142868},
                                                {3.402, 0.154761904761916},
                                                {3.4, 0.152380952380963},
                                                {3.398, 0.150000000000011},
                                                {3.396, 0.147619047619059},
                                                {3.394, 0.145238095238106},
                                                {3.391, 0.142857142857154},
                                                {3.387, 0.140476190476201},
                                                {3.383, 0.138095238095249},
                                                {3.379, 0.135714285714297},
                                                {3.374, 0.133333333333344},
                                                {3.37, 0.130952380952392},
                                                {3.366, 0.128571428571439},
                                                {3.362, 0.126190476190487},
                                                {3.357, 0.123809523809535},
                                                {3.353, 0.121428571428582},
                                                {3.346, 0.11904761904763},
                                                {3.339, 0.116666666666678},
                                                {3.331, 0.114285714285725},
                                                {3.324, 0.111904761904773},
                                                {3.317, 0.10952380952382},
                                                {3.309, 0.107142857142868},
                                                {3.302, 0.104761904761916},
                                                {3.295, 0.102380952380963},
                                                {3.288, 0.100000000000011},
                                                {3.281, 0.0976190476190585},
                                                {3.274, 0.0952380952381061},
                                                {3.267, 0.0928571428571537},
                                                {3.261, 0.0904761904762013},
                                                {3.254, 0.088095238095249},
                                                {3.247, 0.0857142857142966},
                                                {3.239, 0.0833333333333442},
                                                {3.229, 0.0809523809523918},
                                                {3.219, 0.0785714285714394},
                                                {3.209, 0.076190476190487},
                                                {3.198, 0.0738095238095347},
                                                {3.188, 0.0714285714285823},
                                                {3.178, 0.0690476190476299},
                                                {3.167, 0.0666666666666775},
                                                {3.156, 0.0642857142857251},
                                                {3.145, 0.0619047619047728},
                                                {3.134, 0.0595238095238204},
                                                {3.123, 0.057142857142868},
                                                {3.112, 0.0547619047619156},
                                                {3.098, 0.0523809523809632},
                                                {3.085, 0.0500000000000108},
                                                {3.072, 0.0476190476190584},
                                                {3.059, 0.0452380952381061},
                                                {3.045, 0.0428571428571537},
                                                {3.032, 0.0404761904762013},
                                                {3.019, 0.0380952380952489},
                                                {3.004, 0.0357142857142965},
                                                {2.989, 0.0333333333333441},
                                                {2.974, 0.0309523809523918},
                                                {2.959, 0.0285714285714394},
                                                {2.944, 0.026190476190487},
                                                {2.929, 0.0238095238095346},
                                                {2.912, 0.0214285714285822},
                                                {2.895, 0.0190476190476299},
                                                {2.878, 0.0166666666666775},
                                                {2.861, 0.0142857142857251},
                                                {2.842, 0.0119047619047727},
                                                {2.817, 0.00952380952382034},
                                                {2.793, 0.00714285714286796},
                                                {2.768, 0.00476190476191558},
                                                {2.73, 0.0023809523809632},
                                                {2.691, 0}};

    float Interpolate(const std::map<float, float>& map, float x)
    {
        typedef std::map<float, float>::const_iterator iterator;
        iterator ub = map.upper_bound(x);
        if (ub == map.end())
        {
            return (--ub)->second;
        }
        if (ub == map.begin())
        {
            return ub->second;
        }
        iterator lb = ub;
        lb--;
        const float delta = (x - lb->first) / (ub->first - lb->first);
        return delta * ub->second + (1 - delta) * lb->second;
    }
};