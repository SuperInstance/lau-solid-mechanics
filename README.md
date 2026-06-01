# lau-solid-mechanics

**Continuum mechanics in Rust** — stress and strain tensors, constitutive relations, Mohr's circle, beam mechanics, 1D finite elements, yield criteria, plane stress/strain, energy methods, and agent structural analysis.

## What This Does

This crate implements the fundamentals of solid and structural mechanics:

- **Stress and strain tensors** — Cauchy stress tensor and infinitesimal strain tensor as symmetric 3×3 matrices, with hydrostatic/deviatoric decomposition, principal values, invariants, traction vectors, and normal/shear stress on arbitrary planes
- **Constitutive relations** — isotropic Hooke's law (stress↔strain, Lamé parameters, shear/bulk moduli), 6×6 Voigt stiffness and compliance matrices, orthotropic elasticity tensor
- **Mohr's circle** — 2D and 3D Mohr's circle construction, stress transformation at arbitrary angles, principal stresses and maximum shear
- **Beam mechanics** — Euler-Bernoulli beam: deflection, bending moment, and shear force for simply-supported and cantilever beams with point loads, UDLs, and moments
- **Finite elements** — 1D bar elements, global stiffness assembly, boundary condition application, displacement solution, element stress recovery, strain energy, equilibrium verification
- **Yield criteria** — von Mises (equivalent stress, yield check, safety factor) and Tresca (maximum shear stress criterion)
- **Plane stress and plane strain** — 2D stress-strain conversion for both conditions, out-of-plane response, 3×3 stiffness matrices
- **Energy methods** — strain energy density (from stress, strain, or both), bar and beam strain energy, Castigliano's theorem for displacements, complementary energy
- **Agent structural analysis** — maps agent architecture components to structural analogies for stress testing, pipeline analysis via FEM, beam-based latency modeling, weakest-component detection, health scoring

## Key Idea

Continuum mechanics provides a mathematical framework for describing how materials deform and fail under load. This library encodes the standard toolkit: you build a stress state, apply a constitutive law to get strain (or vice versa), check yield criteria, and compute derived quantities. The beam and FEM modules extend this to structural elements.

Everything uses `nalgebra` matrices, is `Serialize`/`Deserialize`, and the test suite verifies results against known analytical solutions.

## Install

```toml
[dependencies]
lau-solid-mechanics = "0.1.0"
```

Requires Rust 2021 edition. Depends on `nalgebra` (linear algebra) and `serde` (serialization).

## Quick Start

### Stress Tensor and Principal Stresses

```rust
use lau_solid_mechanics::tensor::StressTensor;

let sigma = StressTensor::from_components(80.0, -40.0, 20.0, 30.0, 0.0, 0.0);
println!("Hydrostatic stress: {}", sigma.hydrostatic());
println!("Von Mises: {}", sigma.von_mises());
println!("Principal stresses: {:?}", sigma.principal_stresses());
println!("Max shear: {}", sigma.max_shear());
```

### Hooke's Law (Stress → Strain → Stress Round-Trip)

```rust
use lau_solid_mechanics::{StressTensor, StrainTensor, HookeIsotropic};

let steel = HookeIsotropic::new(200e9, 0.3);
let stress = StressTensor::uniaxial(100e6);
let strain = steel.strain_from_stress(&stress);
let stress_back = steel.stress_from_strain(&strain);
// stress ≈ stress_back
```

### Mohr's Circle

```rust
use lau_solid_mechanics::MohrCircle;

let mc = MohrCircle::from_2d(80.0, -40.0, 30.0);
println!("σ₁ = {}, σ₂ = {}", mc.sigma1, mc.sigma2);
println!("τ_max = {}, θ_p = {} rad", mc.tau_max, mc.theta_p);

// Stress state at 45°
let (sn, tn) = mc.stress_at_angle(std::f64::consts::FRAC_PI_4);
```

### Beam Deflection

```rust
use lau_solid_mechanics::EulerBernoulliBeam;

let beam = EulerBernoulliBeam::new(200e9, 1e-4, 5.0);
let p = 10000.0;
println!("Max deflection (centered load): {}", beam.max_deflection_centered_point(p));
println!("Max bending moment: {}", beam.max_bending_moment_point(p, 2.5));
let (rl, rr) = beam.reactions_simply_supported_point(p, 2.5);
```

### 1D Finite Element Analysis

```rust
use lau_solid_mechanics::fem::{BarElement1D, FemAssembler1D};

let fem = FemAssembler1D::uniform_bar(200e9, 1e-4, 5.0, 10);
let mut forces = vec![0.0; 11];
forces[10] = 50000.0;
let u = fem.solve(&forces, &[0]).unwrap();
println!("Tip displacement: {} m", u[10]);
let stresses = fem.element_stresses(&u);
let energy = fem.total_strain_energy(&u);
```

### Yield Criteria

```rust
use lau_solid_mechanics::{VonMises, Tresca, StressTensor};

let vm = VonMises::new(250e6);
let stress = StressTensor::from_components(200e6, 100e6, -50e6, 60e6, 0.0, 0.0);
println!("Von Mises equivalent: {}", VonMises::equivalent_stress(&stress));
println!("Safety factor: {}", vm.safety_factor(&stress));
println!("Yielded? {}", vm.has_yielded(&stress));
```

### Plane Stress / Plane Strain

```rust
use lau_solid_mechanics::{PlaneStress, PlaneStrain, HookeIsotropic};

let mat = HookeIsotropic::new(200e9, 0.3);
let ps = PlaneStress::new(mat.clone());
let (exx, eyy, gxy) = ps.strain_from_stress_2d(100e6, 50e6, 20e6);
let (sxx, syy, txy) = ps.stress_from_strain_2d(exx, eyy, gxy);
```

### Agent Structural Analysis

```rust
use lau_solid_mechanics::agent_analysis::{AgentStructuralAnalysis, AgentComponent};

let mut analysis = AgentStructuralAnalysis::new(1000.0, 500.0);
analysis.add_component(AgentComponent {
    name: "LLM".into(), stiffness: 1000.0, capacity: 500.0, throughput: 10.0,
});
analysis.add_component(AgentComponent {
    name: "Memory".into(), stiffness: 800.0, capacity: 300.0, throughput: 5.0,
});
let results = analysis.analyze_all(&[200.0, 350.0]);
let weakest = analysis.find_weakest(&[200.0, 350.0]);
let health = analysis.health_score(&[200.0, 350.0]);
```

## API Reference

### `tensor` — Stress and Strain

| Type / Fn | Description |
|-----------|-------------|
| `StressTensor` | Symmetric 3×3 Cauchy stress tensor |
| `.from_components(σ_xx, σ_yy, σ_zz, τ_xy, τ_xz, τ_yz)` | Build from 6 independent components |
| `.uniaxial / .biaxial / .pure_shear` | Common stress states |
| `.hydrostatic()` | Mean stress σ_m = trace/3 |
| `.deviatoric()` | s = σ − σ_m I |
| `.von_mises()` | Equivalent stress σ_vm = √(3/2 s:s) |
| `.principal_stresses()` | Eigenvalues in descending order |
| `.max_shear()` | (σ₁ − σ₃)/2 |
| `.first/second/third_invariant()` | Stress invariants I₁, I₂, I₃ |
| `.traction(normal)` | Traction vector t = σ·n |
| `.normal_stress_on_plane(n)` / `.shear_stress_on_plane(n)` | Stress components on a plane |
| `StrainTensor` | Symmetric 3×3 infinitesimal strain |
| `.volumetric()` | ε_v = trace |
| `.from_engineering(exx, eyy, ezz, γ_xy, γ_xz, γ_yz)` | From engineering shear strains |

### `constitutive` — Hooke's Law

| Type / Fn | Description |
|-----------|-------------|
| `HookeIsotropic` | Isotropic linear elastic material (E, ν) |
| `.shear_modulus()` | G = E / (2(1+ν)) |
| `.bulk_modulus()` | K = E / (3(1−2ν)) |
| `.lame_lambda()` / `.lame_mu()` | Lamé parameters |
| `.stress_from_strain(ε)` / `.strain_from_stress(σ)` | 3D Hooke's law |
| `.stiffness_matrix_voigt()` / `.compliance_matrix_voigt()` | 6×6 matrices |
| `ElasticityTensor` | General 6×6 Voigt stiffness |
| `.orthotropic(E₁,E₂,E₃,ν₁₂,ν₁₃,ν₂₃,G₁₂,G₁₃,G₂₃)` | Build orthotropic material |

### `mohr` — Mohr's Circle

| Type / Fn | Description |
|-----------|-------------|
| `MohrCircle` | 2D Mohr's circle (center, radius, principal stresses, angle) |
| `.from_2d(σ_xx, σ_yy, τ_xy)` | Construct from 2D stress state |
| `.from_3d(stress)` | Three Mohr's circles for 3D |
| `.stress_at_angle(θ)` | (σ_n, τ_n) at rotation angle |
| `.absolute_max_shear(stress)` | 3D maximum shear |
| `stress_transform_2d(σ_xx, σ_yy, τ_xy, θ)` | Rotate stress state by θ |

### `beam` — Euler-Bernoulli Beam

| Type / Fn | Description |
|-----------|-------------|
| `EulerBernoulliBeam` | Prismatic beam with loads |
| `.deflection_simply_supported_point(x, P, a)` | v(x) for SS beam with off-center load |
| `.max_deflection_centered_point(P)` | PL³/(48EI) |
| `.bending_moment_simply_supported_point(x, P, a)` | M(x) |
| `.shear_force_simply_supported_point(x, P, a)` | V(x) |
| `.reactions_simply_supported_point(P, a)` | (R_left, R_right) |
| `.deflection_cantilever_point_end(x, P)` | Cantilever with end load |
| `.deflection_cantilever_udl(x, w)` | Cantilever with UDL |
| `.bending_stress(M, y)` | σ = My/I |
| `BeamLoad::Point / Distributed / Moment` | Load types |

### `fem` — 1D Finite Elements

| Type / Fn | Description |
|-----------|-------------|
| `BarElement1D` | Two-node axial bar element |
| `.stiffness()` | EA/L |
| `.local_stiffness_matrix()` | 2×2 [k, −k; −k, k] |
| `.axial_stress(u_i, u_j)` | σ = E(u_j − u_i)/L |
| `FemAssembler1D` | Multi-element assembler |
| `.uniform_bar(E, A, L, n)` | Discretize into n elements |
| `.assemble_global_stiffness()` | Global K matrix |
| `.solve(forces, fixed_nodes)` | Solve Ku = F → displacements |
| `.element_stresses(u)` | Stress recovery |
| `.total_strain_energy(u)` | Σ 0.5 k Δu² |
| `.check_equilibrium(u, F)` | ‖Ku − F‖ |

### `yield_criteria` — Von Mises and Tresca

| Type / Fn | Description |
|-----------|-------------|
| `VonMises::new(σ_y)` | Von Mises criterion |
| `.equivalent_stress(σ)` / `.has_yielded(σ)` / `.safety_factor(σ)` | Evaluate |
| `Tresca::from_tensile(σ_y)` | Tresca criterion |
| `.max_shear(σ)` / `.has_yielded(σ)` / `.safety_factor(σ)` | Evaluate |

### `plane` — Plane Stress and Plane Strain

| Type / Fn | Description |
|-----------|-------------|
| `PlaneStress` | σ_zz = 0 (thin structures) |
| `.strain_from_stress_2d / .stress_from_strain_2d` | 2D conversion |
| `.out_of_plane_strain(σ_xx, σ_yy)` | ε_zz = −ν(σ_xx + σ_yy)/E |
| `.stiffness_matrix_2d()` | 3×3 plane stress stiffness |
| `PlaneStrain` | ε_zz = 0 (thick structures) |
| `.out_of_plane_stress(σ_xx, σ_yy)` | σ_zz = ν(σ_xx + σ_yy) |
| `.stiffness_matrix_2d()` | 3×3 plane strain stiffness |

### `energy` — Strain Energy and Castigliano's Theorem

| Type / Fn | Description |
|-----------|-------------|
| `StrainEnergy::density_3d(σ, ε)` | u = 0.5 σ:ε |
| `.density_from_stress(σ, mat)` | u via compliance |
| `.density_from_strain(ε, mat)` | u via stiffness |
| `.bar_axial(F, L, E, A)` | F²L/(2EA) |
| `.beam_bending_simply_supported_centered(P,L,E,I)` | P²L³/(96EI) |
| `.beam_bending_cantilever_end(P,L,E,I)` | P²L³/(6EI) |
| `.castigliano_displacement_bar(F,L,E,A)` | δ = FL/(EA) |
| `.castigliano_deflection_beam_ss_centered(P,L,E,I)` | PL²/(48EI) |

### `agent_analysis` — Agent Structural Analysis

| Type / Fn | Description |
|-----------|-------------|
| `AgentComponent` | Component with stiffness, capacity, throughput |
| `AgentStructuralAnalysis` | Analysis tool with material model and yield criterion |
| `.analyze_component(comp, load)` | Stress, deformation, safety factor, yielded? |
| `.analyze_all(loads)` | Batch analysis |
| `.analyze_as_beam(...)` | Beam-based latency model |
| `.analyze_pipeline(n, load)` | FEM pipeline stress distribution |
| `.find_weakest(loads)` | Component with lowest safety factor |
| `.health_score(loads)` | Average SF penalized for yielded components |

## How It Works

1. **Tensors are symmetric 3×3 matrices.** The `StressTensor` auto-symmetrizes on construction. Principal stresses come from `nalgebra`'s symmetric eigenvalue decomposition. The deviatoric decomposition separates volumetric (hydrostatic) and distortional (deviatoric) response.

2. **Hooke's law uses Lamé parameters.** σ = λ tr(ε) I + 2μ ε, where λ and μ are computed from E and ν. The 6×6 Voigt matrices use the ordering [σ_xx, σ_yy, σ_zz, τ_yz, τ_xz, τ_xy].

3. **Mohr's circle is pure geometry.** For 2D: center = (σ_xx + σ_yy)/2, radius = √(((σ_xx − σ_yy)/2)² + τ_xy²). The principal angle is θ_p = ½ arctan(2τ_xy / (σ_xx − σ_yy)).

4. **Beam formulas are closed-form.** The simply-supported and cantilever solutions are the standard Euler-Bernoulli results from strength of materials (no shear deformation). Deflection is 4th-order: EI v'''' = w(x).

5. **FEM assembles element stiffness into global K.** Each 1D bar contributes its 2×2 [k, −k; −k, k] to the global matrix. Boundary conditions remove fixed DOFs, then Ku = F is solved via LU decomposition. The method converges to the analytical solution for any mesh density (trivially, since bar elements are exact for constant-strain problems).

## The Math

**Cauchy Stress Tensor:** σ is a symmetric second-order tensor with 6 independent components. Principal stresses are the eigenvalues. Invariants: I₁ = tr(σ), I₂ = ½(tr²σ − tr(σ²)), I₃ = det(σ).

**Von Mises Equivalent Stress:** σ_vm = √(3/2) · ‖s‖ where s = σ − (tr σ/3)I is the deviatoric stress. For uniaxial tension σ, σ_vm = σ. For pure shear τ, σ_vm = √3 · τ.

**Hooke's Law (Isotropic):** σᵢⱼ = λ εₖₖ δᵢⱼ + 2μ εᵢⱼ. Compliance: εᵢⱼ = (1+ν)/E · σᵢⱼ − ν/E · σₖₖ δᵢⱼ.

**Mohr's Circle (2D):** Center C = (σ_x + σ_y)/2, radius R = √((σ_x − σ_y)²/4 + τ_xy²). Principal stresses: C ± R. The stress state at angle θ is parameterized as σ_n = C + R cos 2θ, τ_n = −R sin 2θ.

**Euler-Bernoulli Beam:** v''''(x) = w(x)/(EI). For simply-supported with centered point load: δ_max = PL³/(48EI). For cantilever with end load: δ_max = PL³/(3EI).

**FEM Bar Element:** Stiffness k = EA/L. Local matrix [k, −k; −k, k]. Global assembly: Kᵢᵢ += k, Kᵢⱼ += −k, etc. Strain energy: U = ½ uᵀ K u.

**Plane Stress (σ_zz = 0):** Stiffness: E/(1−ν²) · [[1,ν,0],[ν,1,0],[0,0,(1−ν)/2]]. Out-of-plane strain: ε_zz = −ν(σ_xx + σ_yy)/E.

**Plane Strain (ε_zz = 0):** Uses effective Lamé parameters with 1−2ν in the denominator. Out-of-plane stress: σ_zz = ν(σ_xx + σ_yy).

**Castigliano's Theorem:** Displacement at load point = ∂U/∂F. For linear elastic systems, U = F²L/(2EA) → δ = ∂U/∂F = FL/(EA).

## Test Suite

61 tests across all modules:
- Stress tensor: symmetry, hydrostatic, deviatoric, principal stresses, max shear, von Mises, traction
- Strain tensor: volumetric, deviatoric trace
- Constitutive: shear/bulk moduli, stress-strain round-trip, stiffness-compliance inverse, orthotropic creation
- Mohr's circle: pure shear, uniaxial, biaxial, principal angle verification, 3D Mohr, stress at 45°
- Beam: SS centered deflection, cantilever deflection, bending moment, reactions, bending stress, UDL, zero at supports
- FEM: element stiffness, single bar, uniform bar convergence, uniform stress, strain energy, equilibrium
- Yield criteria: von Mises uniaxial/pure shear/biaxial/yield check/safety, Tresca uniaxial/yield/safety, VM vs Tresca comparison
- Plane: stress-strain round-trip (plane stress & strain), out-of-plane quantities, stiffness symmetry
- Energy: uniaxial, stress=strain density equality, bar/beam energy, Castigliano, complementary energy
- Agent analysis: component analysis, yield detection, beam analogy, pipeline, weakest component, health score

## License

MIT
