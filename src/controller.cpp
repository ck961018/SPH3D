#include "controller.h"
#include "resource_manager.h"
#include "point_renderer.h"
#include "triangle_renderer.h"
#include "skybox_renderer.h"

#include "mfd.hpp"
#include "camera.hpp"

PointRenderer* pointRenderer;
TriangleRenderer* triangleRenderer;
SkyboxRenderer* skyboxRenderer;

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
std::ofstream logFile;

// sph参数
double targetSpacing = 0.08;
int numberOfFrames = 100000;
double fps = 60.0;
Frame frame(0, 1.0 / fps);
PciSphSolver3Ptr solver;

// 网格重建参数
std::string method = "zhu_bridson";
double kernelRadius = 0.07;
Vector3D gridSpacing(0.04, 0.04, 0.04);

unsigned int loadCubemap(std::vector<std::string> faces);

Controller::Controller(GLuint width, GLuint height)
    : State(CONTROLLER_ACTIVE), Keys(), Width(width), Height(height) {}

Controller::~Controller() {
  delete pointRenderer;
  delete triangleRenderer;
  delete skyboxRenderer;
}

void Controller::Init() {
  // 加载着色器
  ResourceManager::LoadShader("shaders/point.vs", "shaders/point.fs", nullptr, "point");
  ResourceManager::LoadShader("shaders/triangle.vs", "shaders/triangle.fs", nullptr, "triangle");
  ResourceManager::LoadShader("shaders/skybox.vs", "shaders/skybox.fs", nullptr, "skybox");

  // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glm::mat4 projection =
      glm::perspective(glm::radians(camera.Zoom), (float)Width / (float)Height, 0.1f, 100.0f);
  glm::mat4 view = camera.GetViewMatrix();
  glm::mat4 model = glm::mat4(1.0f);

  ResourceManager::GetShader("point").Use();
  ResourceManager::GetShader("point").SetMatrix4("projection", projection);
  ResourceManager::GetShader("point").SetMatrix4("view", view);
  ResourceManager::GetShader("point").SetMatrix4("model", model);

  ResourceManager::GetShader("triangle").Use();
  ResourceManager::GetShader("triangle").SetMatrix4("projection", projection);
  ResourceManager::GetShader("triangle").SetMatrix4("view", view);
  ResourceManager::GetShader("triangle").SetMatrix4("model", model);
  ResourceManager::GetShader("triangle").SetVector3f("cameraPos", camera.Position);
  ResourceManager::GetShader("triangle").SetInteger("skybox", 0);

  view = glm::mat4(glm::mat3(camera.GetViewMatrix()));
  ResourceManager::GetShader("skybox").Use();
  ResourceManager::GetShader("skybox").SetMatrix4("view", view);
  ResourceManager::GetShader("skybox").SetMatrix4("projection", projection);
  ResourceManager::GetShader("skybox").SetInteger("skybox", 0);

  std::vector<std::string> faces{
      R"(resources/textures/skybox/right.jpg)", R"(resources/textures/skybox/left.jpg)",
      R"(resources/textures/skybox/top.jpg)",   R"(resources/textures/skybox/bottom.jpg)",
      R"(resources/textures/skybox/front.jpg)", R"(resources/textures/skybox/back.jpg)"};
  GLuint cubemapTexture = loadCubemap(faces);

  pointRenderer = new PointRenderer(ResourceManager::GetShader("point"));
  triangleRenderer =
      new TriangleRenderer(ResourceManager::GetShader("triangle"), 200000, cubemapTexture);
  skyboxRenderer = new SkyboxRenderer(ResourceManager::GetShader("skybox"), cubemapTexture);

  // 设置log输出流
  std::string logFilename = R"(log.txt)";
  logFile.open(logFilename.c_str());
  if (logFile) {
    Logging::setAllStream(&logFile);
  }
  init(solver, targetSpacing, numberOfFrames, fps);
}

void Controller::Update(GLfloat dt) {
  if (frame.index == numberOfFrames) {
    return;
  }
  solver->update(frame);
  frame++;
}

void Controller::ProcessInput(GLfloat dt) {
  if (this->Keys[GLFW_KEY_W]) {
    camera.move(UP, dt);
  }
  if (this->Keys[GLFW_KEY_S]) {
    camera.move(DOWN, dt);
  }
  if (this->Keys[GLFW_KEY_A]) {
    camera.move(LEFT, dt);
  }
  if (this->Keys[GLFW_KEY_D]) {
    camera.move(RIGHT, dt);
  }

  if (this->KeysMouse[GLFW_MOUSE_BUTTON_LEFT]) {
    auto dis = camera.Front;
    dis += glm::vec3{0, 0, 3};
    camera.Position += dis;
    auto offset = NowMousePos - OldMousePos;
    camera.rotate(-offset.x, offset.y);
    camera.Position -= dis;
  }
  if (this->KeysMouse[GLFW_MOUSE_BUTTON_RIGHT]) {
    auto offset = NowMousePos - OldMousePos;
    if (offset.x < 0.0) {
      camera.move(FORWARD, dt);
    } else {
      camera.move(BACKWARD, dt);
    }
  }

  glm::mat4 projection =
      glm::perspective(glm::radians(camera.Zoom), (float)Width / (float)Height, 0.1f, 100.0f);
  glm::mat4 view = camera.GetViewMatrix();
  glm::mat4 model = glm::mat4(1.0f);

  ResourceManager::GetShader("point").Use();
  ResourceManager::GetShader("point").SetMatrix4("projection", projection);
  ResourceManager::GetShader("point").SetMatrix4("view", view);
  ResourceManager::GetShader("point").SetMatrix4("model", model);

  ResourceManager::GetShader("triangle").Use();
  ResourceManager::GetShader("triangle").SetMatrix4("projection", projection);
  ResourceManager::GetShader("triangle").SetMatrix4("view", view);
  ResourceManager::GetShader("triangle").SetMatrix4("model", model);
  ResourceManager::GetShader("triangle").SetVector3f("cameraPos", camera.Position);

  view = glm::mat4(glm::mat3(camera.GetViewMatrix()));
  ResourceManager::GetShader("skybox").Use();
  ResourceManager::GetShader("skybox").SetMatrix4("view", view);
  ResourceManager::GetShader("skybox").SetMatrix4("projection", projection);
}

void Controller::Render() {
  auto tmp = solver->particleSystemData();
  std::vector<glm::vec3> particles;
  // tmp->positions().forEach([&](const auto& p) { particles.push_back({p.x, p.y, p.z}); });
  // pointRenderer->draw(particles, 3.0, {0.0, 1.0, 0.0});

  TriangleMesh3 mesh;
  particlesToTriangles(tmp->positions(), gridSpacing, kernelRadius, method, mesh);
  particles.clear();
  for (int i = 0; i < mesh.numberOfPoints(); i++) {
    auto _point = mesh.point(i);
    particles.push_back({_point.x, _point.y, _point.z});
  }

  std::vector<glm::vec3> normal;
  for (int i = 0; i < mesh.numberOfNormals(); i++) {
    auto _normal = mesh.normal(i);
    normal.push_back({_normal.x, _normal.y, _normal.z});
  }

  std::vector<glm::ivec3> positionIndex, normalIndex;
  for (int i = 0; i < mesh.numberOfTriangles(); i++) {
    auto _positionIndex = mesh.pointIndex(i);
    auto _normalIndex = mesh.normalIndex(i);
    positionIndex.push_back({_positionIndex.x, _positionIndex.y, _positionIndex.z});
    normalIndex.push_back({_normalIndex.x, _normalIndex.y, _normalIndex.z});
  }

  // printf("tris: %d\n", mesh.numberOfTriangles());

  triangleRenderer->draw(particles, positionIndex, normal, normalIndex);

  skyboxRenderer->draw();
}

unsigned int loadCubemap(std::vector<std::string> faces) {
  unsigned int textureID;
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

  int width, height, nrChannels;
  for (unsigned int i = 0; i < faces.size(); i++) {
    // printf("%s\n", faces[i].c_str());
    unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
    if (data) {
      glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, data);
      stbi_image_free(data);
    } else {
      printf("wrong image: %I32d", i);
      stbi_image_free(data);
    }
  }
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

  return textureID;
}