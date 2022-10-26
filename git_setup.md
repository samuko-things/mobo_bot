…or create a new repository on the command line

echo "# obot" >> README.md
git init
git add .
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:samuko-things/mobo_bot.git
git push -u origin main




…or push an existing repository from the command line

git remote add origin git@github.com:samuko-things/mobo_bot.git
git branch -M main
git push -u origin main