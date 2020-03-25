/*
 * @file simdyn.hpp
 *
 */
#include <vector>
#include <mutex>


using fdyn_type = bool (*)(
		std::vector<double>&,
		const std::vector<double>&,
		const std::vector<double>&,
		double, void* param);

using fmeas_type = bool (*)(
		std::vector<double>&,
		const std::vector<double>&,
		const std::vector<double>&);

using fperf_type = bool (*)(
		std::vector<double>&,
		const std::vector<double>& x,
		const std::vector<double>& u);

class SimDyn {
	public:
		SimDyn(int Nx, int Nu, int Ny, int Nz,
				fdyn_type, fmeas_type, fperf_type);
		~SimDyn();

		void assing_f(fdyn_type);
		void assing_g(fmeas_type);
		void assing_h(fperf_type);

		/**
		 * Update the control input to the plant
		 */
		bool set_U(const std::vector<double>& u);

        /**
         * Set State
         */
        bool set_X(const std::vector<double>& x);
        
		/**
		 * Get the measurement 
		 */
		bool get_Y(std::vector<double>& y) const;

		/**
		 * Get the current state
		 */
		bool get_X(std::vector<double>& x) const;

		/**
		 * Get the performance value 
		 */
		bool get_Z(std::vector<double>& z) const;

		/**
		 * Simulation step function:
		 * - Update the system state
		 * - Update the measurement 
		 *
		 */
		bool sim_step(double dt, void* pparam);

	private:
		int StateDim_;
		int InputDim_;
		int MeasDim_;
		int PerfDim_;

		mutable std::mutex data_mx_;

		std::vector<double> X_;
		std::vector<double> U_;
		std::vector<double> Y_;
		std::vector<double> Z_;

		/**
		 * Pointer to the dynamics update function
		 */
		bool (*dyn_fun)(std::vector<double>& x_,
				const std::vector<double>& x,
				const std::vector<double>& u,
				double dt, void* p);

		/**
		 * Pointer to the measurement model function
		 */
		bool (*meas_fun)(std::vector<double>& y,
				const std::vector<double>& x,
				const std::vector<double>& u);

		/**
		 * Pointer to the performance evaluation function
		 */
		bool (*perf_fun)(std::vector<double>& z,
				const std::vector<double>& x,
				const std::vector<double>& u);

};
