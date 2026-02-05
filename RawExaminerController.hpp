/// Cogra --- Coburg Graphics Framework 2017 -- 2026
/// (C) by Quirin Meyer
/// quirin.meyer@hs-coburg.de
#pragma once
#include <array>
#include <cmath>
class RawExaminerController
{
public:
  RawExaminerController(bool gazeIntoPositiveZDirection, float defaultTx = 0.0f, float defaultTy = 0.0f,
                        float defaultTz = 0.0f)
      : m_radius(0.8f)
      , m_hemisphere(gazeIntoPositiveZDirection ? -1.0f : 1.0f)
      , m_translationX(defaultTx)
      , m_translationY(defaultTy)
      , m_translationZ(defaultTz)
      , m_defaultTranslationX(defaultTx)
      , m_defaultTranslationY(defaultTy)
      , m_defaultTranslationZ(defaultTz)
  {
    reset();
  }

  void reset()
  {
    m_bRotate = false;
    m_bShift  = false;
    m_bPitch  = false;

    m_rotation = {1, 0, 0, 0};

    m_translationX = m_defaultTranslationX;
    m_translationY = m_defaultTranslationY;
    m_translationZ = m_defaultTranslationZ;

    m_pitchX = m_pitchY = 0.0f;
    m_shiftX = m_shiftY = 0.0f;
  }

  void click(bool pressed, int button, bool keyboardModifier, float mx, float my)
  {
    if (button == 1)
    {
      if (pressed)
      {
        if (keyboardModifier)
        {
          m_bShift  = true;
          m_bRotate = false;
          m_shiftX  = mx;
          m_shiftY  = my;
        }
        else
        {
          m_bRotate = true;
          m_bShift  = false;

          // Inline startRotation
          m_lastX = mx;
          m_lastY = my;
        }
      }
      else
      {
        m_bRotate = false;
        m_bShift  = false;
      }
    }

    if (button == 2)
    {
      if (pressed)
      {
        m_bPitch = true;
        m_pitchX = mx;
        m_pitchY = my;
      }
      else
      {
        m_bPitch = false;
      }
    }
  }

  void move(float mx, float my)
  {
    if (m_bRotate)
    {
      float x0 = m_lastX;
      float y0 = m_lastY;

      float x1 = mx;
      float y1 = my;

      // Project to hemisphere
      auto project = [&](float x, float y)
      {
        float d = x * x + y * y;
        if (d <= m_radius * m_radius)
          return std::array<float, 3> {x, y, m_hemisphere * std::sqrt(m_radius * m_radius - d)};
        else
          return std::array<float, 3> {x, y, 0.0f};
      };

      auto p0 = project(x0, y0);
      auto p1 = project(x1, y1);

      // Quaternion from arc
      auto arcQuat = [&](const std::array<float, 3>& a, const std::array<float, 3>& b)
      {
        float cx = a[1] * b[2] - a[2] * b[1];
        float cy = a[2] * b[0] - a[0] * b[2];
        float cz = a[0] * b[1] - a[1] * b[0];
        float d  = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        return std::array<float, 4> {d, cx, cy, cz};
      };

      auto q = arcQuat(p0, p1);

      // Quaternion multiply
      auto mul = [&](const std::array<float, 4>& A, const std::array<float, 4>& B)
      {
        float w1 = A[0], x1 = A[1], y1 = A[2], z1 = A[3];
        float w2 = B[0], x2 = B[1], y2 = B[2], z2 = B[3];

        return std::array<float, 4> {w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2};
      };

      m_rotation = mul(q, m_rotation);

      // Normalize
      auto norm = [&](std::array<float, 4>& q)
      {
        float L = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (L == 0)
        {
          q = {1, 0, 0, 0};
          return;
        }
        float inv = 1.0f / L;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
      };

      norm(m_rotation);

      m_lastX = mx;
      m_lastY = my;
    }

    if (m_bPitch)
    {
      float dx = mx - m_pitchX;
      float dy = my - m_pitchY;
      m_translationX += dx;
      m_translationY += dy;
      m_pitchX = mx;
      m_pitchY = my;
    }

    if (m_bShift)
    {
      float dy = my - m_shiftY;
      m_translationZ -= dy;
      m_shiftX = mx;
      m_shiftY = my;
    }
  }

  void abort()
  {
    m_bRotate = false;
    m_bShift  = false;
    m_bPitch  = false;
  }

  bool active() const
  {
    return m_bRotate || m_bShift || m_bPitch;
  }

  std::array<float, 16> getRotationMatrix() const
  {
    float w = m_rotation[0], x = m_rotation[1], y = m_rotation[2], z = m_rotation[3];

    return {1 - 2 * (y * y + z * z),
            2 * (x * y + z * w),
            2 * (x * z - y * w),
            0,
            2 * (x * y - z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z + x * w),
            0,
            2 * (x * z + y * w),
            2 * (y * z - x * w),
            1 - 2 * (x * x + y * y),
            0,
            0,
            0,
            0,
            1};
  }

  std::array<float, 16> getTranslationMatrix() const
  {
    return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, m_translationX, m_translationY, m_translationZ, 1};
  }

  std::array<float, 16> getTransformationMatrix() const
  {
    auto r = getRotationMatrix();
    r[12]  = m_translationX;
    r[13]  = m_translationY;
    r[14]  = m_translationZ;
    return r;
  }

  std::array<float, 4> getRotationQuaternion() const
  {
    return m_rotation;
  }

  void setRotationQuaternion(const std::array<float, 4>& q)
  {
    m_rotation = q;
  }

  std::array<float, 3> getTranslationVector() const
  {
    return {m_translationX, m_translationY, m_translationZ};
  }

  void setTranslationVector(const std::array<float, 3>& t)
  {
    m_translationX = t[0];
    m_translationY = t[1];
    m_translationZ = t[2];
  }

private:
  // Trackball state
  float                m_radius;
  float                m_lastX = 0.0f;
  float                m_lastY = 0.0f;
  float                m_hemisphere;
  std::array<float, 4> m_rotation;

  // Interaction state
  bool m_bRotate = false;
  bool m_bShift  = false;
  bool m_bPitch  = false;

  // Pitch/Shift state
  float m_pitchX = 0.0f;
  float m_pitchY = 0.0f;
  float m_shiftX = 0.0f;
  float m_shiftY = 0.0f;

  // Translation
  float m_translationX;
  float m_translationY;
  float m_translationZ;

  float m_defaultTranslationX;
  float m_defaultTranslationY;
  float m_defaultTranslationZ;
};
