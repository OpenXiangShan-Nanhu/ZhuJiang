import mill._
import scalalib._
import scalafmt._
import $file.`rocket-chip`.common
import $file.`rocket-chip`.dependencies.hardfloat.common
import $file.`rocket-chip`.dependencies.cde.common

val defaultVersions = Map(
  "chisel" -> "6.7.0",
  "chisel-plugin" -> "6.7.0",
  "chiseltest" -> "6.0.0",
  "scala" -> "2.13.16",
  "scalatest" -> "3.2.7"
)

def getVersion(dep: String, org: String = "org.chipsalliance", cross: Boolean = false) = {
  val version = sys.env.getOrElse(dep + "Version", defaultVersions(dep))
  if(cross)
    ivy"$org:::$dep:$version"
  else
    ivy"$org::$dep:$version"
}

trait CommonModule extends ScalaModule {
  override def scalaVersion = defaultVersions("scala")
  override def scalacPluginIvyDeps = Agg(getVersion("chisel-plugin", cross = true))
  override def scalacOptions = super.scalacOptions() ++ Agg("-Ymacro-annotations", "-Ytasty-reader")
  override def ivyDeps = super.ivyDeps() ++ Agg(
    getVersion("chisel"),
    getVersion("chiseltest", "edu.berkeley.cs"),
    ivy"org.chipsalliance:llvm-firtool:1.62.1"
  )
}

object cde extends CommonModule {
  override def millSourcePath = os.pwd / "rocket-chip" / "dependencies" / "cde" / "cde"
}

object diplomacy extends CommonModule {
  override def millSourcePath = os.pwd / "rocket-chip" / "dependencies" / "diplomacy" / "diplomacy"
  override def moduleDeps = super.moduleDeps ++ Seq(cde)
  def sourcecodeIvy = ivy"com.lihaoyi::sourcecode:0.3.1"
}

object rocketchip extends millbuild.`rocket-chip`.common.RocketChipModule {
  def scalaVersion: T[String] = T(defaultVersions("scala"))
  override def millSourcePath = os.pwd / "rocket-chip"
  def chiselModule = None
  def chiselPluginJar = None
  def chiselIvy = Some(getVersion("chisel"))
  def chiselPluginIvy = Some(getVersion("chisel-plugin", cross = true))
  def macrosModule = macros
  def hardfloatModule = hardfloat
  def cdeModule = cde
  def diplomacyModule = diplomacy
  def diplomacyIvy = None
  def mainargsIvy = ivy"com.lihaoyi::mainargs:0.5.0"
  def json4sJacksonIvy = ivy"org.json4s::json4s-jackson:4.0.5"

  object hardfloat extends CommonModule {
    override def millSourcePath = os.pwd / "rocket-chip" / "dependencies" / "hardfloat" / "hardfloat"
  }

  object macros extends millbuild.`rocket-chip`.common.MacrosModule with SbtModule {
    def scalaVersion: T[String] = T(defaultVersions("scala"))
    def scalaReflectIvy = ivy"org.scala-lang:scala-reflect:${defaultVersions("scala")}"
  }
}

object xsutils extends SbtModule with CommonModule {
  override def millSourcePath = os.pwd / "xs-utils"
  override def moduleDeps = super.moduleDeps ++ Seq(rocketchip, cde)
}

object zhujiang extends SbtModule with ScalafmtModule with CommonModule {

  override def millSourcePath = os.pwd

  override def moduleDeps = super.moduleDeps ++ Seq(rocketchip, xsutils)

  override def ivyDeps = super.ivyDeps() ++ Agg(
    getVersion("chisel"),
    getVersion("chiseltest", "edu.berkeley.cs"),
    ivy"org.chipsalliance:llvm-firtool:1.62.1"
  )

  object test extends SbtModuleTests with TestModule.ScalaTest {
    override def ivyDeps = super.ivyDeps() ++ Agg(
      getVersion("scalatest", "org.scalatest")
    )

    def testFramework = "org.scalatest.tools.Framework"
  }
}