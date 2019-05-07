
#include "distributed_mapper/run_distributed_mapper.h"
#include <opencv2/core/core.hpp>
using namespace std;
using namespace gtsam;

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config () {} // private constructor makes a singleton
public:
    ~Config()
    {
        if ( file_.isOpened() )
            file_.release();
    }// close the file when deconstructing

    // set a new config file
    static void setParameterFile( const std::string& filename )
    {
        if ( config_ == nullptr )
            config_ = shared_ptr<Config>(new Config);
        config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
        if ( config_->file_.isOpened() == false )
        {
            std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
            config_->file_.release();
            return;
        }
    }

    // access the parameter values
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
};
shared_ptr<Config> Config::config_ = nullptr;

int main(int argc, char *argv[]) {

    Config::setParameterFile ( argv[1] );
    ///////////////////////////////////////////////////////////////////
    //Command line arguments
    ///////////////////////////////////////////////////////////////////
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("/media/D/robust_distributed_mapper/test_data/datasets/city10000/spilt_part/");
    //string data_dir("/media/D/robust_distributed_mapper/test_data/pairwise_consistency_maximization/clean/simulation/example_2robots/");// data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;

    ///////////////////////////////////////////////////////////////////
    // Config (specify noise covariances and maximum number iterations)
    ///////////////////////////////////////////////////////////////////
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 10000;  // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1;  // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Diffe2ence between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.95; // confidence probability for the pairwise consistency computation.
    bool use_covariance = true; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    bool use_PCMHeu = false;
    bool use_DCS = false;
    double DCS_phir = 1.0;
    double DCS_phip = 1.0;

    data_dir = Config::get<string>("Dataset.dir");
    int use_PCM_index = Config::get<int>("PCM");
    if(use_PCM_index)  use_PCM = true;
    confidence_probability = Config::get<double>("Confidence.probability");
    rotation_estimate_change_threshold = Config::get<double>("Rotation.threshold");
    pose_estimate_change_threshold = Config::get<double>("Pose.threshold");
    max_iter = Config::get<int>("Max.iter");
    int use_DCS_index = Config::get<int>("DCS");// USE DCS robust kernel
    if(use_DCS_index)  use_DCS = true;
    DCS_phir = Config::get<double>("DCS.phir"); // DCS parameters set
    DCS_phip = Config::get<double>("DCS.phip");

    int use_PCMHeu_index = Config::get<int>("PCMHeu");
    if(use_PCMHeu_index) use_PCMHeu = true;

    try {
        // Parse program options
        namespace po = boost::program_options;
        po::options_description desc("Options");
        desc.add_options()
                ("help", "Print help messages")
                ("nr_robots, n", po::value<size_t>(&nr_robots), "number of robots (default: 2)")
                ("data_dir, l", po::value<string>(&data_dir), "data directory (default: /tmp)")
                ("trace_file, t", po::value<string>(&trace_file), "trace file (default: runG2o)")
                ("log_dir, l", po::value<string>(&log_dir), "log directory (default: /tmp)")
                ("use_XY, u", po::value<bool>(&use_XY), "use x,y,z as naming convention or a,b,c (default: x,y,z)")
                ("use_OP, o", po::value<bool>(&use_OP), "use o,p,q as naming convention (default: x,y,z)")
                ("use_flagged_init, f", po::value<bool>(&use_flagged_init),
                 "use flagged initialization or not (default: true)")
                ("use_between_noise, b", po::value<bool>(&use_between_noise),
                 "use the given factor between noise instead of unit noise(default: false)")
                ("use_chr_less_full_graph", po::value<bool>(&use_chr_less_full_graph),
                 "whether full graph has character indexes or not (default: false)")
                ("use_landmarks, l", po::value<bool>(&use_landmarks), "use landmarks or not (default: false)")
                ("rthresh, r", po::value<double>(&rotation_estimate_change_threshold),
                 "Specify difference between rotation estimate provides an early stopping condition (default: 1e-2)")
                ("pthresh, p", po::value<double>(&pose_estimate_change_threshold),
                 "Specify difference between pose estimate provides an early stopping condition (default: 1e-2)")
                ("max_iter, m", po::value<size_t>(&max_iter), "maximum number of iterations (default: 100000)")
                ("confidence, c", po::value<double>(&confidence_probability), "confidence probability for the pairwise consistency computation (default: 0.99)")
                ("use_covariance, i", po::value<bool>(&use_covariance), "use covariance in dataset file (default: false)")
                ("debug, d", po::value<bool>(&debug), "debug (default: false)")
                ("use_PCM", po::value<bool>(&use_PCM), "use pairwise consistency maximization (default: true)");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
            if (vm.count("help")) { // --help option
                cout << "Run Distributed-Mapper" << endl
                     << "Example: ./rung2o --data_dir ../../../example/ --num_robots 4"
                     << endl << desc << endl;
                return 0;
            }
            po::notify(vm);  // throws on error, so do after help in case
        }
        catch (po::error &e) {
            cerr << "ERROR: " << e.what() << endl << endl;
            cerr << desc << endl;
            return 1;
        }
    }
    catch (exception &e) {
        cerr << "Unhandled Exception reached the top of main: "
             << e.what() << ", application will now exit" << endl;
        return 2;
    }
    std::tuple<double, double, int> results = distributed_mapper::runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
            use_XY, use_OP, debug, priorModel, model,
            max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
            gamma, use_flagged_init, update_type, use_between_noise,
            use_chr_less_full_graph, use_landmarks, confidence_probability, use_covariance, use_PCM, use_DCS, DCS_phir,DCS_phip, use_PCMHeu);

    return 0;
}