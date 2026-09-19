# BOOT.uvproj generator: derive from LED.uvproj template (ASCII only)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$src  = Join-Path $root 'USER\LED.uvproj'
$dst  = Join-Path $PSScriptRoot 'BOOT.uvproj'

$content = Get-Content $src -Raw -Encoding UTF8

# 1) target name / output name
$content = $content.Replace('<TargetName>LED</TargetName>', '<TargetName>BOOT</TargetName>')
$content = $content.Replace('<OutputName>LED</OutputName>', '<OutputName>BOOT</OutputName>')

# 2) output dir -> BOOT\OBJ
$content = $content.Replace('<OutputDirectory>..\OBJ\</OutputDirectory>', '<OutputDirectory>.\OBJ\</OutputDirectory>')
$content = $content.Replace('<ListingPath>..\OBJ\</ListingPath>', '<ListingPath>.\OBJ\</ListingPath>')

# 3) IROM size 1MB -> 16KB (linker + target memory, 2 places)
$content = $content.Replace('<Size>0x100000</Size>', '<Size>0x4000</Size>')

# 4) include path
$oldInc = '<IncludePath>..\CORE;..\SYSTEM\delay;..\SYSTEM\sys;..\SYSTEM\usart;..\USER;..\HARDWARE\LED;..\FWLIB\inc;..\HARDWARE\PWM;..\HARDWARE\ESP8266;..\HARDWARE\LCD;..\HARDWARE\CAN;..\HARDWARE\TOUCH;..\HARDWARE\FreeRTOS;..\FreeRTOS\Source\include;..\FreeRTOS\Source\portable\GCC\ARM_CM4F;..\HARDWARE\LVGL;..\HARDWARE\LVGL\src;..\HARDWARE\LVGL\port</IncludePath>'
$newInc = '<IncludePath>..\CORE;..\USER;..\FWLIB\inc;.</IncludePath>'
if (-not $content.Contains($oldInc)) { Write-Host 'WARN: IncludePath not matched!' }
$content = $content.Replace($oldInc, $newInc)

# 5) replace Groups block
$gs = $content.IndexOf('<Groups>')
$ge = $content.LastIndexOf('</Groups>') + '</Groups>'.Length
if ($gs -lt 0 -or $ge -lt 0) { throw 'Groups block not found' }

$newGroups = @'
      <Groups>
        <Group>
          <GroupName>BOOT</GroupName>
          <Files>
            <File>
              <FileName>main.c</FileName>
              <FileType>1</FileType>
              <FilePath>.\main.c</FilePath>
            </File>
            <File>
              <FileName>boot_flash.c</FileName>
              <FileType>1</FileType>
              <FilePath>.\boot_flash.c</FilePath>
            </File>
            <File>
              <FileName>boot_uart.c</FileName>
              <FileType>1</FileType>
              <FilePath>.\boot_uart.c</FilePath>
            </File>
            <File>
              <FileName>crc32.c</FileName>
              <FileType>1</FileType>
              <FilePath>.\crc32.c</FilePath>
            </File>
          </Files>
        </Group>
        <Group>
          <GroupName>SYSTEM</GroupName>
          <Files>
            <File>
              <FileName>system_stm32f4xx.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\USER\system_stm32f4xx.c</FilePath>
            </File>
          </Files>
        </Group>
        <Group>
          <GroupName>CORE</GroupName>
          <Files>
            <File>
              <FileName>startup_stm32f40_41xxx.s</FileName>
              <FileType>2</FileType>
              <FilePath>..\CORE\startup_stm32f40_41xxx.s</FilePath>
            </File>
          </Files>
        </Group>
        <Group>
          <GroupName>FWLIB</GroupName>
          <Files>
            <File>
              <FileName>misc.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\misc.c</FilePath>
            </File>
            <File>
              <FileName>stm32f4xx_rcc.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\stm32f4xx_rcc.c</FilePath>
            </File>
            <File>
              <FileName>stm32f4xx_gpio.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\stm32f4xx_gpio.c</FilePath>
            </File>
            <File>
              <FileName>stm32f4xx_usart.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\stm32f4xx_usart.c</FilePath>
            </File>
            <File>
              <FileName>stm32f4xx_flash.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\stm32f4xx_flash.c</FilePath>
            </File>
            <File>
              <FileName>stm32f4xx_iwdg.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\FWLIB\src\stm32f4xx_iwdg.c</FilePath>
            </File>
          </Files>
        </Group>
      </Groups>
'@

$content = $content.Substring(0, $gs) + $newGroups + $content.Substring($ge)

Set-Content $dst $content -Encoding UTF8
Write-Host 'OK: BOOT.uvproj generated'
