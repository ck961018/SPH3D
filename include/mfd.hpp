#include <jet/jet.h>

using namespace jet;

const std::string kSpherical = "spherical";
const std::string kSph = "sph";
const std::string kZhuBridson = "zhu_bridson";
const std::string kAnisotropic = "anisotropic";

double sSphCutOffDensity = 0.3;
double sZhuBridsonCutOffThreshold = 0.8;
double sAnisoCutOffDensity = 0.3;
double sAnisoPositionSmoothingFactor = 0.7;
size_t sAnisoMinNumNeighbors = 10;

BoundingBox3D domain(Vector3D(-1, -1, 0), Vector3D(1, 1, 0.4));

void init(PciSphSolver3Ptr& solver, double targetSpacing, int numberOfFrames, double fps) {
  BoundingBox3D domain2(Vector3D(-0.01, -1, 0), Vector3D(0.01, -0.7, 0.4));

  // Build solver
  solver = PciSphSolver3::builder()
               .withTargetDensity(100.0)
               .withTargetSpacing(targetSpacing)
               .withRelativeKernelRadius(1.8)
               .makeShared();

  solver->setPseudoViscosityCoefficient(0);

  // Build emitter
  BoundingBox3D sourceBound(domain);
  sourceBound.expand(-targetSpacing);

  auto sphere = Sphere3::builder()
                    .withCenter(domain.midPoint())
                    .withRadius(0.35 * domain.width())
                    .makeShared();

  auto surfaceSet = ImplicitSurfaceSet3::builder().withExplicitSurfaces({sphere}).makeShared();

  auto emitter = VolumeParticleEmitter3::builder()
                     .withImplicitSurface(surfaceSet)
                     .withSpacing(targetSpacing)
                     .withMaxRegion(sourceBound)
                     .withIsOneShot(true)
                     .makeShared();

  solver->setEmitter(emitter);

  // Build collider
  auto box = Box3::builder().withIsNormalFlipped(true).withBoundingBox(domain).makeShared();
  auto box2 = Box3::builder().withIsNormalFlipped(false).withBoundingBox(domain2).makeShared();
  auto surfaceSet2 = ImplicitSurfaceSet3::builder().withExplicitSurfaces({box, box2}).makeShared();

  auto collider = RigidBodyCollider3::builder().withSurface(surfaceSet2).makeShared();

  solver->setCollider(collider);
}

void printInfo(const Size2& resolution, const BoundingBox2D& domain, const Vector2D& gridSpacing,
               size_t numberOfParticles, const std::string& method) {
  printf("Resolution: %zu x %zu\n", resolution.x, resolution.y);
  printf("Domain: [%f, %f] x [%f, %f]\n", domain.lowerCorner.x, domain.lowerCorner.y,
         domain.upperCorner.x, domain.upperCorner.y);
  printf("Grid spacing: [%f, %f]\n", gridSpacing.x, gridSpacing.y);
  printf("Number of particles: %zu\n", numberOfParticles);
  printf("Reconstruction method: %s\n", method.c_str());
}

void particlesToTriangles(const ArrayAccessor1<Vector3D>& positions, const Vector3D& gridSpacing,
                          double kernelRadius, const std::string& method, TriangleMesh3& mesh) {
  Size3 resolution(static_cast<size_t>(domain.width() / gridSpacing.x),
                   static_cast<size_t>(domain.height() / gridSpacing.y),
                   static_cast<size_t>(domain.depth() / gridSpacing.z));
  PointsToImplicit3Ptr converter;
  if (method == kSpherical) {
    converter = std::make_shared<SphericalPointsToImplicit3>(kernelRadius, false);
  } else if (method == kSph) {
    converter = std::make_shared<SphPointsToImplicit3>(kernelRadius, sSphCutOffDensity, false);
  } else if (method == kZhuBridson) {
    converter = std::make_shared<ZhuBridsonPointsToImplicit3>(kernelRadius,
                                                              sZhuBridsonCutOffThreshold, false);
  } else {
    converter = std::make_shared<AnisotropicPointsToImplicit3>(kernelRadius, sAnisoCutOffDensity,
                                                               sAnisoPositionSmoothingFactor,
                                                               sAnisoMinNumNeighbors, false);
  }

  VertexCenteredScalarGrid3 sdf(resolution, gridSpacing, domain.lowerCorner);
  // printInfo(resolution, sdf.boundingBox(), gridSpacing, positions.size(), method);

  converter->convert(positions, &sdf);

  marchingCubes(sdf.dataAccessor(), sdf.gridSpacing(), sdf.dataOrigin(), &mesh, 0.0, kDirectionAll);
}