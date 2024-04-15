// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <fmt/format.h>

#include <Eigen/Eigenvalues>

#include "frc/EigenCore.h"
#include "frc/system/Discretization.h"

#include "wpinet/EventLoopRunner.h"
#include "wpinet/HttpServerConnection.h"
#include "wpinet/UrlParser.h"
#include "wpinet/uv/Loop.h"
#include "wpinet/uv/Tcp.h"

namespace uv = wpi::uv;

class MyHttpServerConnection : public wpi::HttpServerConnection {
public:
  explicit MyHttpServerConnection(std::shared_ptr<uv::Stream> stream)
      : HttpServerConnection(stream) {}

protected:
  void ProcessRequest() override;
};

void MyHttpServerConnection::ProcessRequest() {
  fmt::print(stderr, "HTTP request: '{}'\n", m_request.GetUrl());
  wpi::UrlParser url{m_request.GetUrl(),
                     m_request.GetMethod() == wpi::HTTP_CONNECT};
  if (!url.IsValid()) {
    // failed to parse URL
    SendError(400);
    return;
  }

  std::string_view path;
  if (url.HasPath()) {
    path = url.GetPath();
  }
  fmt::print(stderr, "path: \"{}\"\n", path);

  std::string_view query;
  if (url.HasQuery()) {
    query = url.GetQuery();
  }
  fmt::print(stderr, "query: \"{}\"\n", query);

  const bool isGET = m_request.GetMethod() == wpi::HTTP_GET;
  if (isGET && path == "/") {
    // build HTML root page
    SendResponse(200, "OK", "text/html",
                 "<html><head><title>WebServer Example</title></head>"
                 "<body><p>This is an example root page from the webserver."
                 "</body></html>");
  } else {
    SendError(404, "Resource not found");
  }
}

int main() {
  Robot robot;

  frc::Matrixd<2, 2> contA{{0, 1}, {0, 0}};
  frc::Matrixd<2, 1> contB{0, 1};

  frc::Vectord<2> x0{1, 1};
  frc::Vectord<1> u{1};
  frc::Matrixd<2, 2> discA;
  frc::Matrixd<2, 1> discB;

  frc::DiscretizeAB<2, 1>(contA, contB, 1_s, &discA, &discB);
  frc::Vectord<2> x1Discrete = discA * x0 + discB * u;

  // We now have pos = vel = accel = 1, which should give us:
  frc::Vectord<2> x1Truth{1.0 * x0(0) + 1.0 * x0(1) + 0.5 * u(0),
                          0.0 * x0(0) + 1.0 * x0(1) + 1.0 * u(0)};

  fmt::print("X1Truth {}\n", x1Truth(1));

    wpi::EventLoopRunner loop;
  loop.ExecAsync([](uv::Loop &loop) {
    auto tcp = uv::Tcp::Create(loop);

    // bind to listen address and port
    tcp->Bind("", 8080);

    // when we get a connection, accept it and start reading
    tcp->connection.connect([srv = tcp.get()] {
      auto tcp = srv->Accept();
      if (!tcp) {
        return;
      }
      std::fputs("Got a connection\n", stderr);
      auto conn = std::make_shared<MyHttpServerConnection>(tcp);
      tcp->SetData(conn);
    });

    // start listening for incoming connections
    tcp->Listen();

    std::fputs("Listening on port 8080\n", stderr);
  });

  // wait for a keypress to terminate
  return 0;
}