-- Project-specific Neovim configuration

vim.lsp.config('yamlls',
{
  settings = {
    yaml = {
      schemas = {
        ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMStates.json"] = "src/states/*.yaml",
        ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_rbdyn/RobotModule.json"] = "data/tools/yaml/*.yaml"
      },
      validate = true,
      format = { enable = false },
      hover = true,
      completion = true,
    }
  }
})
