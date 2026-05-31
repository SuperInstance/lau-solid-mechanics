//! Agent structural analysis — stress testing agent architectural configurations.
//!
//! Maps agent architecture concepts to structural mechanics analogies for
//! stress testing and resilience analysis.

use serde::{Deserialize, Serialize};

use crate::constitutive::HookeIsotropic;
use crate::tensor::StressTensor;
use crate::yield_criteria::VonMises;
use crate::energy::StrainEnergy;
use crate::beam::EulerBernoulliBeam;
use crate::fem::FemAssembler1D;

/// An agent component in the structural analogy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentComponent {
    /// Name of the component (e.g., "LLM layer", "memory", "tool interface")
    pub name: String,
    /// Stiffness (resistance to perturbation) — analogous to Young's modulus
    pub stiffness: f64,
    /// Capacity (load-bearing ability) — analogous to yield stress
    pub capacity: f64,
    /// Cross-sectional area analogy (bandwidth/throughput)
    pub throughput: f64,
}

/// Result of structural analysis on an agent configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisResult {
    /// Component name
    pub component: String,
    /// Applied load (stress)
    pub applied_load: f64,
    /// Resulting deformation
    pub deformation: f64,
    /// Safety factor
    pub safety_factor: f64,
    /// Whether the component has yielded
    pub yielded: bool,
    /// Strain energy absorbed
    pub energy_absorbed: f64,
}

/// Agent structural analysis tool.
pub struct AgentStructuralAnalysis {
    /// Material analogy for the agent
    pub material: HookeIsotropic,
    /// Yield criterion
    pub yield_criterion: VonMises,
    /// Components
    pub components: Vec<AgentComponent>,
}

impl AgentStructuralAnalysis {
    /// Create a new analysis for an agent configuration.
    pub fn new(base_modulus: f64, yield_stress: f64) -> Self {
        Self {
            material: HookeIsotropic::new(base_modulus, 0.3),
            yield_criterion: VonMises::new(yield_stress),
            components: Vec::new(),
        }
    }

    /// Add a component.
    pub fn add_component(&mut self, component: AgentComponent) {
        self.components.push(component);
    }

    /// Analyze stress on a single component under given load.
    pub fn analyze_component(&self, component: &AgentComponent, load: f64) -> AnalysisResult {
        let stress = StressTensor::uniaxial(load);
        let strain = self.material.strain_from_stress(&stress);
        let safety = self.yield_criterion.safety_factor(&stress);
        let yielded = self.yield_criterion.has_yielded(&stress);
        let energy = StrainEnergy::density_from_stress(&stress, &self.material);

        AnalysisResult {
            component: component.name.clone(),
            applied_load: load,
            deformation: strain.matrix[(0, 0)],
            safety_factor: safety,
            yielded,
            energy_absorbed: energy,
        }
    }

    /// Analyze all components under given loads.
    pub fn analyze_all(&self, loads: &[f64]) -> Vec<AnalysisResult> {
        self.components.iter().zip(loads.iter())
            .map(|(comp, &load)| self.analyze_component(comp, load))
            .collect()
    }

    /// Model the agent as a beam under distributed load (request rate).
    /// Returns the maximum deflection (latency under load).
    pub fn analyze_as_beam(
        &self,
        length: f64,
        moment_of_inertia: f64,
        request_rate: f64,  // N/m analogy
        max_request_size: f64,  // Point load analogy
    ) -> (f64, f64) {
        let beam = EulerBernoulliBeam::new(self.material.youngs_modulus, moment_of_inertia, length);
        let defl_udl = beam.max_deflection_cantilever_udl(request_rate);
        let defl_point = beam.max_deflection_cantilever_point_end(max_request_size);
        (defl_udl, defl_point)
    }

    /// Model the agent as an FEM bar (pipeline of components).
    pub fn analyze_pipeline(&self, n_stages: usize, end_load: f64) -> Vec<f64> {
        let fem = FemAssembler1D::uniform_bar(
            self.material.youngs_modulus,
            1.0,
            n_stages as f64,
            n_stages,
        );
        let mut forces = vec![0.0; n_stages + 1];
        forces[n_stages] = end_load;
        let u = fem.solve(&forces, &[0]).unwrap();
        fem.element_stresses(&u)
    }

    /// Find the weakest component (lowest safety factor).
    pub fn find_weakest(&self, loads: &[f64]) -> Option<(usize, AnalysisResult)> {
        let results = self.analyze_all(loads);
        results.into_iter().enumerate()
            .min_by(|(_, a), (_, b)| a.safety_factor.partial_cmp(&b.safety_factor).unwrap())
    }

    /// Overall health score: average safety factor, penalized for any yielded components.
    pub fn health_score(&self, loads: &[f64]) -> f64 {
        let results = self.analyze_all(loads);
        if results.is_empty() {
            return f64::INFINITY;
        }
        let avg_sf: f64 = results.iter().map(|r| r.safety_factor).sum::<f64>() / results.len() as f64;
        let yielded_count = results.iter().filter(|r| r.yielded).count() as f64;
        avg_sf / (1.0 + yielded_count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_agent_analysis_single_component() {
        let mut analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
        analysis.add_component(AgentComponent {
            name: "LLM".to_string(),
            stiffness: 1000.0,
            capacity: 500.0,
            throughput: 10.0,
        });
        let result = analysis.analyze_component(&analysis.components[0], 200.0);
        assert!(!result.yielded);
        assert!(result.safety_factor > 1.0);
    }

    #[test]
    fn test_agent_yield_detection() {
        let mut analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
        analysis.add_component(AgentComponent {
            name: "Memory".to_string(),
            stiffness: 1000.0,
            capacity: 500.0,
            throughput: 5.0,
        });
        let result = analysis.analyze_component(&analysis.components[0], 600.0);
        assert!(result.yielded);
    }

    #[test]
    fn test_agent_beam_analysis() {
        let analysis = AgentStructuralAnalysis::new(200e9, 250e6);
        let (udl, point) = analysis.analyze_as_beam(1.0, 1e-4, 1000.0, 5000.0);
        assert!(udl > 0.0);
        assert!(point > 0.0);
    }

    #[test]
    fn test_agent_pipeline() {
        let analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
        let stresses = analysis.analyze_pipeline(5, 100.0);
        assert_eq!(stresses.len(), 5);
        // All should have same stress for uniform bar
        for &s in &stresses {
            assert_relative_eq!(s, stresses[0], epsilon = 1e-6);
        }
    }

    #[test]
    fn test_weakest_component() {
        let mut analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
        analysis.add_component(AgentComponent {
            name: "Strong".to_string(), stiffness: 1000.0, capacity: 500.0, throughput: 10.0,
        });
        analysis.add_component(AgentComponent {
            name: "Weak".to_string(), stiffness: 1000.0, capacity: 500.0, throughput: 5.0,
        });
        let loads = [200.0, 450.0];
        let (idx, result) = analysis.find_weakest(&loads).unwrap();
        assert_eq!(idx, 1);
        assert_eq!(result.component, "Weak");
    }

    #[test]
    fn test_health_score() {
        let mut analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
        analysis.add_component(AgentComponent {
            name: "A".to_string(), stiffness: 1000.0, capacity: 500.0, throughput: 1.0,
        });
        let score = analysis.health_score(&[200.0]);
        assert!(score > 1.0);
    }
}
